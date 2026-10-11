#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ==================================================
// BNO055 (lying flat on top of the cuboid)
// ==================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ==================================================
// ESP32-S3 MOTOR PINS  (same roles as before)
//
//  Bottom motor  10/11 (L298N)  : roll / twist left-right   (uses HEADING)
//  Motor         4/5   (L9110S) : tilt left-right           (uses ROLL  or PITCH)
//  Motor         6/7   (L9110S) : tilt front-back           (uses PITCH or ROLL)
// ==================================================
#define TWIST_A 10
#define TWIST_B 11

#define LR_A 4
#define LR_B 5

#define FB_A 6
#define FB_B 7

// false: tilt left/right = BNO "roll" (y), tilt front/back = BNO "pitch" (z)
// true : swapped. Change this if the wrong motor reacts when you tilt.
#define SWAP_TILT_AXES false

// ==================================================
// SETTINGS
// ==================================================
// Per-axis PWM limits live in the Axis lines below.
// The L298N (twist motor) loses ~2 V inside, so it needs a much higher
// start / max PWM than the L9110S drivers.
const int TEST_PWM = 150;          // used by the 't' motor test

const float STABLE_ZONE  = 0.9;    // deg: motor off inside this angle ...
const float RATE_ZONE    = 3.0;    // deg/s: ... and when slower than this
const float D_FILTER     = 0.3;    // 0..1, smaller = smoother derivative
const float INTEG_LIMIT  = 30.0;   // anti-windup limit for the integral
const float TUMBLE_LIMIT = 60.0;   // deg: motors switch off beyond this

const unsigned long CONTROL_MS = 10;     // 100 Hz control loop
const unsigned long DISPLAY_MS = 1000;
const int HOME_SECONDS = 10;

bool motorsEnabled = true;   // set false to start OFF. 'e' = enable, 'd' = disable (serial)
bool plotMode = true;       // 'g' = Serial Plotter output
bool telemetry = true;       // 'v' = toggle data stream for the HTML interface
unsigned long lastTele = 0, lastCal = 0;
uint8_t calS = 0, calG = 0, calA = 0;

// ==================================================
// AXIS STRUCT
// sign: +1 / -1. If a motor pushes the WRONG way (error grows),
// flip its sign here.
// ==================================================
struct Axis {
  const char* name;
  uint8_t pinA, pinB;
  float kp, ki, kd;
  int8_t sign;
  float err, prevErr, dFilt, integ;
  int cmd;
  int startPwm, maxPwm;   // minimum push to start the motor, and its limit
};

//                name     pinA    pinB    Kp   Ki   Kd    sign  err prev d  int cmd
Axis twist  = { "TWIST ", TWIST_A, TWIST_B, 1.5, 0.0, 0.20, +1,   0,  0,   0, 0,  0, 110, 160 };
Axis tiltLR = { "TILT-LR", LR_A,   LR_B,    1.5, 0.0, 0.20, +1,   0,  0,   0, 0,  0, 55, 70 };
Axis tiltFB = { "TILT-FB", FB_A,   FB_B,    1.5, 0.0, 0.20, +1,   0,  0,   0, 0,  0, 55, 70 };

// ==================================================
// HOME ORIENTATION
// ==================================================
float homeHeading = 0, homeRoll = 0, homePitch = 0;
unsigned long lastControl = 0, lastDisplay = 0;
unsigned int plotCount = 0;

// ==================================================
// HELPERS
// ==================================================
float angleDifference(float target, float current) {
  float e = target - current;
  while (e > 180)  e -= 360;
  while (e < -180) e += 360;
  return e;
}

void driveMotor(uint8_t pinA, uint8_t pinB, int command) {
  command = constrain(command, -255, 255);
  if (command > 0)      { analogWrite(pinA, command); analogWrite(pinB, 0); }
  else if (command < 0) { analogWrite(pinA, 0);       analogWrite(pinB, -command); }
  else                  { analogWrite(pinA, 0);       analogWrite(pinB, 0); }
}

void stopAllMotors() {
  driveMotor(twist.pinA,  twist.pinB,  0);
  driveMotor(tiltLR.pinA, tiltLR.pinB, 0);
  driveMotor(tiltFB.pinA, tiltFB.pinB, 0);
}

// PD(I) controller for one axis.
// error = home - current, so the correct law is
//   output = Kp*error + Ki*integral(error) + Kd*d(error)/dt
void updateAxis(Axis &a, float error, float dt) {
  float d = (error - a.prevErr) / dt;
  a.prevErr = error;
  a.err = error;
  a.dFilt += D_FILTER * (d - a.dFilt);

  // motor off only when the angle AND the speed are small
  if (fabs(error) <= STABLE_ZONE && fabs(a.dFilt) < RATE_ZONE) {
    a.cmd = 0;
    a.integ = 0;
    return;
  }

  a.integ = constrain(a.integ + error * dt, -INTEG_LIMIT, INTEG_LIMIT);

  float out = a.kp * error + a.ki * a.integ + a.kd * a.dFilt;
  out *= a.sign;

  if (out > 0 && out < a.startPwm)       out = a.startPwm;
  else if (out < 0 && out > -a.startPwm) out = -a.startPwm;

  a.cmd = constrain((int)out, -a.maxPwm, a.maxPwm);
}

void saveHome() {
  sensors_event_t e;
  bno.getEvent(&e);
  homeHeading = e.orientation.x;
  homeRoll    = e.orientation.y;
  homePitch   = e.orientation.z;
  Axis* all[3] = { &twist, &tiltLR, &tiltFB };
  for (int i = 0; i < 3; i++) {
    all[i]->err = all[i]->prevErr = all[i]->dFilt = all[i]->integ = 0;
    all[i]->cmd = 0;
  }
  Serial.print("HOME saved  heading=");
  Serial.print(homeHeading, 2);
  Serial.print("  roll=");
  Serial.print(homeRoll, 2);
  Serial.print("  pitch=");
  Serial.println(homePitch, 2);
}

// Motor test: spins each motor forward then reverse, ignoring the sensor.
void motorTest() {
  bool was = motorsEnabled;
  motorsEnabled = false;
  stopAllMotors();
  Axis* all[3] = { &twist, &tiltLR, &tiltFB };
  for (int i = 0; i < 3; i++) {
    Serial.print("TEST "); Serial.print(all[i]->name); Serial.println(" forward");
    driveMotor(all[i]->pinA, all[i]->pinB, TEST_PWM);  delay(900);
    driveMotor(all[i]->pinA, all[i]->pinB, 0);         delay(300);
    Serial.print("TEST "); Serial.print(all[i]->name); Serial.println(" reverse");
    driveMotor(all[i]->pinA, all[i]->pinB, -TEST_PWM); delay(900);
    driveMotor(all[i]->pinA, all[i]->pinB, 0);         delay(300);
  }
  Serial.println("TEST done");
  motorsEnabled = was;
}

void printGains() {
  char b[160];
  snprintf(b, sizeof b, "G,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
           twist.kp, twist.ki, twist.kd, tiltLR.kp, tiltLR.ki, tiltLR.kd,
           tiltFB.kp, tiltFB.ki, tiltFB.kd);
  Serial.println(b);
}

// ==================================================
// SERIAL COMMANDS
//   e  enable motors        d  disable motors
//   h  save new home        g  toggle plotter output
//   Tune live (type then Enter), axis T / L / F:
//     T kp 2.0     L kd 0.35     F ki 0.05
// ==================================================
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') { motorsEnabled = true;  Serial.println("Motors ENABLED"); }
    else if (c == 'd') { motorsEnabled = false; stopAllMotors(); Serial.println("Motors DISABLED"); }
    else if (c == 'h') { saveHome(); }
    else if (c == 'g') { plotMode = !plotMode; }
    else if (c == 'v') { telemetry = !telemetry; }
    else if (c == 'q') { printGains(); }
    else if (c == 't') { motorTest(); }
    else if (c == 'T' || c == 'L' || c == 'F') {
      Axis* a = (c == 'T') ? &twist : (c == 'L') ? &tiltLR : &tiltFB;
      String s = Serial.readStringUntil('\n');
      s.trim();
      float v = s.substring(2).toFloat();
      if (s.startsWith("kp"))      a->kp = v;
      else if (s.startsWith("ki")) a->ki = v;
      else if (s.startsWith("kd")) a->kd = v;
      else if (s.startsWith("mn")) a->startPwm = (int)v;   // e.g.  T mn 110
      else if (s.startsWith("mx")) a->maxPwm = (int)v;     // e.g.  T mx 160
      printGains();
    }
  }
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }   // wait for USB serial (S3)

  uint8_t pins[6] = { TWIST_A, TWIST_B, LR_A, LR_B, FB_A, FB_B };
  for (int i = 0; i < 6; i++) pinMode(pins[i], OUTPUT);
  stopAllMotors();

  Wire.begin(8, 9);   // SDA = GPIO8, SCL = GPIO9
  Wire.setClock(400000);

  Serial.println();
  Serial.println("ESP32-S3 BNO055 + 3 MOTOR STABILIZER");

  // IMU-only mode: no magnetometer, so motor magnets cannot corrupt the heading
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("ERROR: BNO055 not detected!");
    while (true) { stopAllMotors(); delay(100); }
  }
  bno.setExtCrystalUse(true);

  Serial.println("Keep the satellite in its home position.");
  for (int i = HOME_SECONDS; i > 0; i--) {
    stopAllMotors();
    Serial.print("Saving home in ");
    Serial.println(i);
    delay(1000);
  }
  saveHome();

  Serial.println("Motors are DISABLED. Send e to enable.");
  Serial.println("Commands: e / d / h / g, and  T|L|F kp|ki|kd value");
  Serial.println("T = twist (bottom motor 10/11)");
  Serial.println("L = tilt left/right (4/5)");
  Serial.println("F = tilt front/back (6/7)");

  printGains();
  lastControl = lastDisplay = millis();
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastControl < CONTROL_MS) return;
  float dt = (now - lastControl) / 1000.0;
  lastControl = now;

  sensors_event_t ev;
  bno.getEvent(&ev);

  float heading = ev.orientation.x;
  float roll    = ev.orientation.y;
  float pitch   = ev.orientation.z;

  float headingErr = angleDifference(homeHeading, heading);
  float rollErr    = angleDifference(homeRoll, roll);
  float pitchErr   = angleDifference(homePitch, pitch);

  float lrErr = SWAP_TILT_AXES ? pitchErr : rollErr;
  float fbErr = SWAP_TILT_AXES ? rollErr  : pitchErr;

  // safety: if the body tumbles, stop all motors
  if (motorsEnabled && (fabs(headingErr) > TUMBLE_LIMIT ||
                        fabs(lrErr) > TUMBLE_LIMIT ||
                        fabs(fbErr) > TUMBLE_LIMIT)) {
    motorsEnabled = false;
    stopAllMotors();
    Serial.println("TUMBLE LIMIT: motors DISABLED (send 'e' to enable)");
  }

  updateAxis(twist,  headingErr, dt);
  updateAxis(tiltLR, lrErr,      dt);
  updateAxis(tiltFB, fbErr,      dt);

  if (motorsEnabled) {
    driveMotor(twist.pinA,  twist.pinB,  twist.cmd);
    driveMotor(tiltLR.pinA, tiltLR.pinB, tiltLR.cmd);
    driveMotor(tiltFB.pinA, tiltFB.pinB, tiltFB.cmd);
  } else {
    stopAllMotors();
    twist.cmd = tiltLR.cmd = tiltFB.cmd = 0;
  }

  // Data stream for the HTML interface (20 Hz)
  if (telemetry && now - lastTele >= 50) {
    lastTele = now;
    if (now - lastCal >= 500) {
      lastCal = now;
      uint8_t cm;
      bno.getCalibration(&calS, &calG, &calA, &cm);
    }
    char b[170];
    snprintf(b, sizeof b, "$,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%u,%u,%u",
             heading, roll, pitch, twist.err, tiltLR.err, tiltFB.err,
             twist.cmd, tiltLR.cmd, tiltFB.cmd, motorsEnabled ? 1 : 0, calS, calG, calA);
    Serial.println(b);
  }

  // Serial Plotter output (send 'g')
  if (plotMode && !telemetry && ++plotCount >= 5) {
    plotCount = 0;
    Serial.print("Twist:");  Serial.print(twist.err, 2);
    Serial.print(",TiltLR:"); Serial.print(tiltLR.err, 2);
    Serial.print(",TiltFB:"); Serial.println(tiltFB.err, 2);
  }

  // slow status print
  if (!plotMode && !telemetry && now - lastDisplay >= DISPLAY_MS) {
    lastDisplay = now;
    Serial.println();
    Serial.println("======================================");
    Serial.print("TWIST   (10/11) err: "); Serial.print(twist.err, 2);
    Serial.print("  PWM: "); Serial.println(twist.cmd);
    Serial.print("TILT-LR (4/5) err: "); Serial.print(tiltLR.err, 2);
    Serial.print("  PWM: "); Serial.println(tiltLR.cmd);
    Serial.print("TILT-FB (6/7) err: "); Serial.print(tiltFB.err, 2);
    Serial.print("  PWM: "); Serial.println(tiltFB.cmd);
    bool stable = fabs(twist.err) <= STABLE_ZONE &&
                  fabs(tiltLR.err) <= STABLE_ZONE &&
                  fabs(tiltFB.err) <= STABLE_ZONE;
    Serial.println(stable ? "STATUS: HOME / STABLE ZONE" : "STATUS: DISPLACED FROM HOME");
    Serial.print("MOTOR OUTPUT: ");
    Serial.println(motorsEnabled ? "ENABLED" : "DISABLED");
    uint8_t cs, cg, ca, cm;
    bno.getCalibration(&cs, &cg, &ca, &cm);
    Serial.print("CALIB sys/gyro/accel: ");
    Serial.print(cs); Serial.print("/"); Serial.print(cg); Serial.print("/"); Serial.println(ca);
    Serial.println("======================================");
  }
}
