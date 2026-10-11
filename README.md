# 3-Axis-Gyro-Stabilization-Simulation
This project implements a 3‑axis gyroscopic stabilization system with integrated simulation and verification software. It is designed to demonstrate real‑time orientation control, stability analysis, and performance evaluation across roll, pitch, and yaw axes.

# ESP32‑S3 3‑Axis Gyro Stabilization System with Web Simulation

## 📌 Overview
This project implements a **3‑axis gyroscopic stabilization system** using an **ESP32‑S3 microcontroller**, **BNO055 IMU sensor**, and **N20 geared motors**.  
It features a **PC‑based simulation webpage** that receives live telemetry via the COM port, allowing real‑time visualization of roll, pitch, and yaw stabilization.

---

## ⚙️ Hardware Setup
- **ESP32‑S3** (main controller)
- **BNO055 IMU Sensor**  
  - SDA → GPIO21  
  - SCL → GPIO22  
- **Motors & Drivers**  
  - 2 × N20 geared motors + **L9110S driver**  
    - GPIOs: 4, 5, 6, 7  
  - 1 × N20 geared motor + **L298N driver**  
    - GPIOs: 10, 11  
- **Wheels** attached to N20 motors for stabilization movement

---

## 🔌 Wiring Diagram
<img width="1544" height="1028" alt="image" src="https://github.com/user-attachments/assets/9a19d277-c9a6-43b9-9bd4-cfd8b25d0761" />
 
![ESP32‑S3 Gyro Stabilization Wiring Diagram](docs/wiring.png)

---

## 🖥️ Software Features
- Reads orientation data from **BNO055** (roll, pitch, yaw)
- Runs stabilization logic on **ESP32‑S3**
- Streams telemetry via **COM port** to laptop
- **Web‑based interface** for simulation checks and visualization
- Displays altitude vs. time graphs and sensor data (velocity, pressure, temperature)

---

## 🚀 Installation & Setup

### 1. Hardware Wiring
- Connect BNO055 via I²C (GPIO21 SDA, GPIO22 SCL).
- Wire two N20 motors to L9110S driver (GPIO4–7).
- Wire one N20 motor to L298N driver (GPIO10–11).
- Attach wheels to motors for stabilization.

### 2. ESP32‑S3 Firmware
- Install [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/).
- Add **ESP32 board support**.
- Flash the provided firmware (`src/`) to ESP32‑S3.

### 3. Laptop Web Interface
- Open the `web/` folder.
- Run the HTML file in a browser.
- Select the COM port connected to ESP32‑S3.
- Upload CSV telemetry files or view live data.

---

## 🌐 Webpage Interface
<img width="1222" height="1396" alt="image" src="https://github.com/user-attachments/assets/530c2f40-3592-45eb-9e6f-a28294ebd74d" />
  
![Simulation Webpage Interface](docs/webpage.png)  
🔗 [Live Demo / Webpage Link](https://your‑website‑link‑here)

---

## 📊 Simulation Features
- Plots **altitude vs. time** with clear node‑by‑node visualization.
- Displays **velocity, pressure, and temperature** below the graph.
- Supports **dual CSV comparison** (two files side‑by‑side with different colors).
- Tables show raw sensor data for inspection.

---

## 📂 Repository Structure
