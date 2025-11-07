# Sensor Fusion System

A comprehensive ROS2 package for multi-sensor data fusion, logging, and alerting.

## Features

- **Sensor Publisher**: Publishes simulated sensor data (IMU, GPS, Temperature, Barometer)
- **Data Logger**: Logs all sensor data with statistics (min, max, average)
- **Alert System**: Monitors sensors and detects anomalies
- **Sensor Fusion**: Combines multiple sensor data for state estimation
- **Launch Files**: Easy startup of all nodes
- **Unit Tests**: Comprehensive testing of functionality

## System Architecture

```
┌─────────────────┐
│ Sensor Publisher│
│  - IMU (10 Hz)  │
│  - GPS (1 Hz)   │
│  - Temp (2 Hz)  │
│  - Baro (2 Hz)  │
└────────┬────────┘
         │
         ├─────────────┬─────────────┬─────────────┐
         │             │             │             │
         v             v             v             v
┌────────────┐  ┌────────────┐ ┌───────────┐ ┌──────────┐
│ Data Logger│  │Alert System│ │   Fusion  │ │  Other   │
│  - Stats   │  │ - Anomalies│ │ - Combine │ │  Nodes   │
│  - Logging │  │ - Alerts   │ │ - Estimate│ │          │
└────────────┘  └────────────┘ └───────────┘ └──────────┘
```

## Installation

### Prerequisites
- ROS2 (Humble or later)
- Python 3.8+
- Required packages: `sensor_msgs`, `std_msgs`, `geometry_msgs`

### Build Instructions

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> sensor_fusion_system
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_system
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch All Nodes
```bash
ros2 launch sensor_fusion_system sensor_fusion.launch.py
```

### Launch with Custom Parameters
```bash
ros2 launch sensor_fusion_system sensor_fusion.launch.py log_level:=debug alert_cooldown:=3.0
```

### Run Individual Nodes

**Sensor Publisher:**
```bash
ros2 run sensor_fusion_system sensor_publisher
```

**Data Logger:**
```bash
ros2 run sensor_fusion_system data_logger
```

**Alert System:**
```bash
ros2 run sensor_fusion_system alert_system
```

**Sensor Fusion:**
```bash
ros2 run sensor_fusion_system sensor_fusion
```

## Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/sensors/imu` | `sensor_msgs/Imu` | 10 Hz | IMU data (acceleration, gyroscope) |
| `/sensors/gps` | `sensor_msgs/NavSatFix` | 1 Hz | GPS position data |
| `/sensors/temperature` | `sensor_msgs/Temperature` | 2 Hz | Temperature readings |
| `/sensors/barometer` | `sensor_msgs/FluidPressure` | 2 Hz | Barometric pressure |
| `/sensors/fused_state` | `geometry_msgs/PoseStamped` | 5 Hz | Fused sensor state |
| `/sensors/estimated_altitude` | `std_msgs/Float32` | 5 Hz | Estimated altitude |
| `/alerts` | `std_msgs/String` | Variable | Sensor anomaly alerts |

## Configuration

Edit `config/sensor_params.yaml` to customize:
- Publishing rates
- Sensor noise parameters
- Alert thresholds
- Filter parameters

Example:
```yaml
sensor_publisher:
  ros__parameters:
    imu_rate: 10.0
    gps_rate: 1.0

alert_system:
  ros__parameters:
    alert_cooldown: 5.0
    temp_max: 85.0
```

## Testing

Run unit tests:
```bash
cd ~/ros2_ws
colcon test --packages-select sensor_fusion_system
colcon test-result --verbose
```

Run specific test:
```bash
python3 src/sensor_fusion_system/test/test_sensor_publisher.py
```

## Data Logging

The data logger creates JSON log files in the current directory:
- Format: `sensor_log_YYYYMMDD_HHMMSS.json`
- Contains timestamped sensor data
- Statistics printed every 5 seconds

## Alert System

Monitors for:
- **IMU**: High acceleration (>20 m/s²), high angular velocity (>5 rad/s)
- **GPS**: Invalid coordinates, extreme altitudes
- **Temperature**: Out of range (-40°C to 85°C)
- **Barometer**: Extreme pressure values
- **All Sensors**: Rapid changes, NaN values

Alert Severity Levels:
- `INFO`: Minor issues or notifications
- `WARNING`: Out-of-range values
- `CRITICAL`: Invalid data or system failures

## Sensor Fusion

Combines sensor data to provide:
- Filtered position estimates
- Altitude calculation from barometer
- State estimation with moving average filter
- Fusion of GPS and barometric altitude

## Troubleshooting

### No sensor data received
```bash
# Check if publisher is running
ros2 node list

# Check topic list
ros2 topic list

# Monitor specific topic
ros2 topic echo /sensors/imu
```

### Build errors
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select sensor_fusion_system
```

### Permission errors
```bash
# Make Python files executable
chmod +x src/sensor_fusion_system/sensor_fusion_system/*.py
```

## Advanced Features

### QoS Profiles
All nodes use appropriate QoS profiles:
- Sensor data: Best effort, volatile
- Alerts: Reliable, transient local

### Error Handling
- Comprehensive try-catch blocks
- Graceful degradation
- Detailed logging at multiple levels

### Performance
- Efficient data structures (deque)
- Limited history storage
- Optimized callback processing

## Future Enhancements

- [ ] Extended Kalman Filter implementation
- [ ] Real-time data visualization
- [ ] ROS2 bag recording integration
- [ ] Multi-robot support
- [ ] Web dashboard for monitoring
- [ ] Machine learning anomaly detection

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Write/update tests
5. Submit a pull request

## License

MIT License - see LICENSE file for details

## Authors

- Your Name <your.email@example.com>

## Acknowledgments

- ROS2 community
- Sensor fusion algorithms reference
- Open source contributors

## Support

For issues and questions:
- GitHub Issues: <repository_url>/issues
- Email: your.email@example.com
- ROS Answers: https://answers.ros.org

## Version History

- **1.0.0** (2024-11-07)
  - Initial release
  - Basic sensor fusion
  - Data logging and alerting
  - Comprehensive test suite