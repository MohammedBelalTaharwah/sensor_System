# 🛰️ Sensor Fusion System (ROS 2 Jazzy)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A comprehensive ROS 2 package for simulating a multi-sensor suite (IMU, GPS, Temperature, Barometer), logging data, performing real-time filtering (Moving Average), detecting anomalies, and visualizing the results.

This project is built for **ROS 2 Jazzy Jalisco**.

## 👨‍💻 Author
* **Name:** Mohammed Al-Taharwah
* **Email:** mtahrawe685@gmail.com

---

## 🏛️ System Architecture

This system is composed of five interconnected ROS 2 nodes that communicate over topics to create a full simulation and processing pipeline.



### 1. `sensor_publisher_node` (The Simulator)
Simulates a suite of noisy sensors.
* **Publishes:**
    * `/sensors/imu` (`sensor_msgs/Imu`) @ 10 Hz
    * `/sensors/gps` (`sensor_msgs/NavSatFix`) @ 1 Hz
    * `/sensors/temperature` (`sensor_msgs/Temperature`) @ 2 Hz
    * `/sensors/barometer` (`sensor_msgs/FluidPressure`) @ 2 Hz
* **Features:** Noise levels are fully configurable via `config/sensor_params.yaml`.

### 2. `sensor_fusion_node` (The Filter)
Subscribes to raw sensor data and applies a moving average filter to clean it.
* **Subscribes to:**
    * `/sensors/temperature`
    * `/sensors/barometer`
* **Publishes:**
    * `/fused/temperature` (`std_msgs/Float64`): The cleaned, filtered temperature.
    * `/fused/altitude` (`std_msgs/Float64`): Altitude calculated from the cleaned barometer data.
* **Features:** The filter window size is configurable via `config/sensor_params.yaml`.

### 3. `data_logger_node` (The Logger)
Monitors all raw sensor data and prints periodic statistical summaries to the console.
* **Subscribes to:**
    * `/sensors/imu`
    * `/sensors/gps`
    * `/sensors/temperature`
    * `/sensors/barometer`
* **Features:** Prints Min, Max, and Average for all sensors every 5 seconds.

### 4. `alert_system_node` (The Watchdog)
Monitors sensor values against predefined safety thresholds.
* **Subscribes to:**
    * `/sensors/temperature`
    * `/sensors/barometer`
* **Publishes:**
    * `/alerts` (`diagnostic_msgs/DiagnosticStatus`): Publishes a warning if thresholds are breached.
* **Features:** Safety thresholds (e.g., `temp_min_c`) are configurable via `config/sensor_params.yaml`.

### 5. `visualization_node` (The Dashboard)
Provides a real-time `matplotlib` plot comparing the raw, noisy sensor data against the clean, filtered data.
* **Subscribes to:**
    * `/sensors/temperature` (Raw)
    * `/sensors/barometer` (Raw)
    * `/fused/temperature` (Filtered)
    * `/fused/altitude` (Filtered)
* **Features:** Uses `plt.ion()` for non-blocking interactive plotting and displays a legend for clarity.

---

## 📂 File Structure

```bash
sensor_System/
└── src/
    └── sensor_fusion_system/
        ├── config/
        │   └── sensor_params.yaml    # All system parameters
        ├── launch/
        │   └── sensor_fusion.launch.py # Main launch file
        ├── resource/
        │   └── sensor_fusion_system  # Ament resource file
        ├── sensor_fusion_system/
        │   ├── __init__.py
        │   ├── sensor_publisher.py   # Node 1
        │   ├── data_logger.py        # Node 2
        │   ├── alert_system.py       # Node 3
        │   ├── sensor_fusion.py      # Node 4
        │   └── visualization.py    # Node 5
        ├── test/
        │   └── ... (Python tests)
        ├── package.xml               # Package "ID Card"
        ├── setup.cfg
        └── setup.py                  # Python build instructions
