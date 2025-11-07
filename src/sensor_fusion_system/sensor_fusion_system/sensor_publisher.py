#!/usr/bin/env python3
"""
Sensor Publisher Node for Multi-Sensor Fusion System

This node simulates multiple sensors with realistic noise characteristics:
- IMU (Inertial Measurement Unit): 10 Hz
- GPS: 1 Hz
- Temperature Sensor: 2 Hz
- Barometer/Altimeter: 2 Hz

Author: ROS2 Jazzy Implementation
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from std_msgs.msg import Header
import numpy as np
import math


class SensorPublisher(Node):
    """
    Publishes simulated sensor data with configurable noise levels.
    
    This node simulates a complete sensor suite for robotics applications,
    including inertial, position, environmental sensors.
    """

    def __init__(self):
        """Initialize the sensor publisher node with all sensors and parameters."""
        super().__init__('sensor_publisher')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize sensor states
        self._initialize_sensor_states()
        
        # Create QoS profiles
        self._setup_qos_profiles()
        
        # Create publishers
        self._create_publishers()
        
        # Create timers for periodic publishing
        self._create_timers()
        
        self.get_logger().info('Sensor Publisher Node initialized successfully')
        self._log_configuration()

    def _declare_parameters(self):
        """Declare all configurable parameters with default values."""
        # Noise level parameters
        self.declare_parameter('imu_noise_level', 0.01)
        self.declare_parameter('gps_noise_level', 0.0001)
        self.declare_parameter('temp_noise_level', 0.5)
        self.declare_parameter('pressure_noise_level', 100.0)
        
        # Initial position parameters (Amman, Jordan as default)
        self.declare_parameter('initial_latitude', 31.9522)
        self.declare_parameter('initial_longitude', 35.9450)
        self.declare_parameter('initial_altitude', 800.0)
        self.declare_parameter('initial_temperature', 25.0)
        
        # Motion simulation parameters
        self.declare_parameter('enable_motion', True)
        self.declare_parameter('motion_speed', 1.0)

    def _get_parameters(self):
        """Retrieve parameter values from parameter server."""
        self.imu_noise = self.get_parameter('imu_noise_level').value
        self.gps_noise = self.get_parameter('gps_noise_level').value
        self.temp_noise = self.get_parameter('temp_noise_level').value
        self.pressure_noise = self.get_parameter('pressure_noise_level').value
        
        self.init_lat = self.get_parameter('initial_latitude').value
        self.init_lon = self.get_parameter('initial_longitude').value
        self.init_alt = self.get_parameter('initial_altitude').value
        self.init_temp = self.get_parameter('initial_temperature').value
        
        self.enable_motion = self.get_parameter('enable_motion').value
        self.motion_speed = self.get_parameter('motion_speed').value

    def _initialize_sensor_states(self):
        """Initialize internal state variables for all sensors."""
        # IMU state
        self.imu_seq = 0
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 9.81]  # Gravity on Z-axis
        
        # GPS state
        self.gps_seq = 0
        self.current_lat = self.init_lat
        self.current_lon = self.init_lon
        self.current_alt = self.init_alt
        
        # Temperature state
        self.temp_seq = 0
        self.current_temp = self.init_temp
        
        # Barometer state
        self.baro_seq = 0
        self.current_pressure = self._altitude_to_pressure(self.init_alt)
        
        # Time tracking for motion simulation
        self.start_time = self.get_clock().now()

    def _setup_qos_profiles(self):
        """Configure Quality of Service profiles for different data types."""
        # High-frequency IMU data - use BEST_EFFORT for lower latency
        self.imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Low-frequency reliable data - use RELIABLE
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _create_publishers(self):
        """Create publishers for all sensor topics."""
        self.imu_publisher = self.create_publisher(
            Imu, '/imu/data', self.imu_qos)
        
        self.gps_publisher = self.create_publisher(
            NavSatFix, '/gps/fix', self.reliable_qos)
        
        self.temp_publisher = self.create_publisher(
            Temperature, '/temperature/data', self.reliable_qos)
        
        self.baro_publisher = self.create_publisher(
            FluidPressure, '/barometer/data', self.reliable_qos)

    def _create_timers(self):
        """Create timers for periodic sensor data publication."""
        # IMU: 10 Hz (100ms period)
        self.imu_timer = self.create_timer(0.1, self.publish_imu_data)
        
        # GPS: 1 Hz (1000ms period)
        self.gps_timer = self.create_timer(1.0, self.publish_gps_data)
        
        # Temperature: 2 Hz (500ms period)
        self.temp_timer = self.create_timer(0.5, self.publish_temperature_data)
        
        # Barometer: 2 Hz (500ms period)
        self.baro_timer = self.create_timer(0.5, self.publish_barometer_data)

    def _log_configuration(self):
        """Log the current configuration for debugging."""
        self.get_logger().info(f'IMU Noise Level: {self.imu_noise}')
        self.get_logger().info(f'GPS Noise Level: {self.gps_noise}')
        self.get_logger().info(f'Temperature Noise: {self.temp_noise}°C')
        self.get_logger().info(f'Pressure Noise: {self.pressure_noise} Pa')
        self.get_logger().info(
            f'Initial Position: ({self.init_lat}, {self.init_lon}, {self.init_alt}m)')

    def _altitude_to_pressure(self, altitude):
        """
        Convert altitude to atmospheric pressure using barometric formula.
        
        Args:
            altitude (float): Altitude in meters above sea level
            
        Returns:
            float: Atmospheric pressure in Pascals
        """
        # Standard atmospheric pressure at sea level (Pa)
        P0 = 101325.0
        # Temperature lapse rate (K/m)
        L = 0.0065
        # Standard temperature at sea level (K)
        T0 = 288.15
        # Universal gas constant (J/(mol·K))
        R = 8.31447
        # Gravitational acceleration (m/s²)
        g = 9.80665
        # Molar mass of Earth's air (kg/mol)
        M = 0.0289644
        
        # Barometric formula
        pressure = P0 * math.pow((1 - (L * altitude) / T0), (g * M) / (R * L))
        return pressure

    def _add_gaussian_noise(self, value, noise_level):
        """
        Add Gaussian (normal) noise to a sensor reading.
        
        Args:
            value (float): Original sensor value
            noise_level (float): Standard deviation of noise
            
        Returns:
            float: Value with added noise
        """
        return value + np.random.normal(0, noise_level)

    def _simulate_motion(self):
        """
        Simulate realistic robot motion for GPS and IMU.
        
        Creates circular motion pattern with varying altitude.
        """
        if not self.enable_motion:
            return
        
        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Circular motion parameters
        radius = 0.0001  # Approximately 11 meters
        angular_freq = 0.1 * self.motion_speed  # rad/s
        
        # Update position (circular motion)
        self.current_lat = self.init_lat + radius * math.cos(angular_freq * elapsed)
        self.current_lon = self.init_lon + radius * math.sin(angular_freq * elapsed)
        
        # Vary altitude sinusoidally
        self.current_alt = self.init_alt + 10 * math.sin(0.05 * elapsed)
        
        # Update IMU based on motion
        # Angular velocity (turning)
        self.angular_velocity[2] = angular_freq
        
        # Linear acceleration (centripetal)
        speed = radius * 111139 * angular_freq  # Convert to m/s
        centripetal = speed * speed / (radius * 111139)
        self.linear_acceleration[0] = centripetal * math.cos(angular_freq * elapsed)
        self.linear_acceleration[1] = centripetal * math.sin(angular_freq * elapsed)

    def publish_imu_data(self):
        """Publish IMU sensor data at 10 Hz."""
        self._simulate_motion()
        
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Angular velocity (rad/s) with noise
        msg.angular_velocity.x = self._add_gaussian_noise(
            self.angular_velocity[0], self.imu_noise)
        msg.angular_velocity.y = self._add_gaussian_noise(
            self.angular_velocity[1], self.imu_noise)
        msg.angular_velocity.z = self._add_gaussian_noise(
            self.angular_velocity[2], self.imu_noise)
        
        # Linear acceleration (m/s²) with noise
        msg.linear_acceleration.x = self._add_gaussian_noise(
            self.linear_acceleration[0], self.imu_noise)
        msg.linear_acceleration.y = self._add_gaussian_noise(
            self.linear_acceleration[1], self.imu_noise)
        msg.linear_acceleration.z = self._add_gaussian_noise(
            self.linear_acceleration[2], self.imu_noise)
        
        # Orientation (quaternion) - simplified, identity orientation
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        # Covariance matrices (diagonal)
        msg.angular_velocity_covariance[0] = self.imu_noise ** 2
        msg.angular_velocity_covariance[4] = self.imu_noise ** 2
        msg.angular_velocity_covariance[8] = self.imu_noise ** 2
        
        msg.linear_acceleration_covariance[0] = self.imu_noise ** 2
        msg.linear_acceleration_covariance[4] = self.imu_noise ** 2
        msg.linear_acceleration_covariance[8] = self.imu_noise ** 2
        
        self.imu_publisher.publish(msg)
        self.imu_seq += 1

    def publish_gps_data(self):
        """Publish GPS sensor data at 1 Hz."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        
        # Position with noise
        msg.latitude = self._add_gaussian_noise(self.current_lat, self.gps_noise)
        msg.longitude = self._add_gaussian_noise(self.current_lon, self.gps_noise)
        msg.altitude = self._add_gaussian_noise(self.current_alt, self.gps_noise * 10)
        
        # GPS status
        msg.status.status = 0  # STATUS_FIX (valid fix)
        msg.status.service = 1  # SERVICE_GPS
        
        # Position covariance
        msg.position_covariance[0] = self.gps_noise ** 2
        msg.position_covariance[4] = self.gps_noise ** 2
        msg.position_covariance[8] = (self.gps_noise * 10) ** 2
        msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(msg)
        self.gps_seq += 1
        
        if self.gps_seq % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f'GPS: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, '
                f'Alt={msg.altitude:.2f}m')

    def publish_temperature_data(self):
        """Publish temperature sensor data at 2 Hz."""
        msg = Temperature()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temp_sensor_link'
        
        # Simulate temperature variation (diurnal cycle)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        temp_variation = 5.0 * math.sin(0.01 * elapsed)  # ±5°C variation
        
        self.current_temp = self.init_temp + temp_variation
        msg.temperature = self._add_gaussian_noise(self.current_temp, self.temp_noise)
        msg.variance = self.temp_noise ** 2
        
        self.temp_publisher.publish(msg)
        self.temp_seq += 1

    def publish_barometer_data(self):
        """Publish barometer/pressure sensor data at 2 Hz."""
        msg = FluidPressure()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'barometer_link'
        
        # Calculate pressure from current altitude
        self.current_pressure = self._altitude_to_pressure(self.current_alt)
        
        # Add noise
        msg.fluid_pressure = self._add_gaussian_noise(
            self.current_pressure, self.pressure_noise)
        msg.variance = self.pressure_noise ** 2
        
        self.baro_publisher.publish(msg)
        self.baro_seq += 1


def main(args=None):
    """Main function to initialize and run the sensor publisher node."""
    rclpy.init(args=args)
    
    try:
        sensor_publisher = SensorPublisher()
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in sensor publisher: {e}')
    finally:
        if rclpy.ok():
            sensor_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
Sensor Publisher Node for Multi-Sensor Fusion System

This node simulates multiple sensors with realistic noise characteristics:
- IMU (Inertial Measurement Unit): 10 Hz
- GPS: 1 Hz
- Temperature Sensor: 2 Hz
- Barometer/Altimeter: 2 Hz

Author: ROS2 Jazzy Implementation
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from std_msgs.msg import Header
import numpy as np
import math


class SensorPublisher(Node):
    """
    Publishes simulated sensor data with configurable noise levels.
    
    This node simulates a complete sensor suite for robotics applications,
    including inertial, position, environmental sensors.
    """

    def __init__(self):
        """Initialize the sensor publisher node with all sensors and parameters."""
        super().__init__('sensor_publisher')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize sensor states
        self._initialize_sensor_states()
        
        # Create QoS profiles
        self._setup_qos_profiles()
        
        # Create publishers
        self._create_publishers()
        
        # Create timers for periodic publishing
        self._create_timers()
        
        self.get_logger().info('Sensor Publisher Node initialized successfully')
        self._log_configuration()

    def _declare_parameters(self):
        """Declare all configurable parameters with default values."""
        # Noise level parameters
        self.declare_parameter('imu_noise_level', 0.01)
        self.declare_parameter('gps_noise_level', 0.0001)
        self.declare_parameter('temp_noise_level', 0.5)
        self.declare_parameter('pressure_noise_level', 100.0)
        
        # Initial position parameters (Amman, Jordan as default)
        self.declare_parameter('initial_latitude', 31.9522)
        self.declare_parameter('initial_longitude', 35.9450)
        self.declare_parameter('initial_altitude', 800.0)
        self.declare_parameter('initial_temperature', 25.0)
        
        # Motion simulation parameters
        self.declare_parameter('enable_motion', True)
        self.declare_parameter('motion_speed', 1.0)

    def _get_parameters(self):
        """Retrieve parameter values from parameter server."""
        self.imu_noise = self.get_parameter('imu_noise_level').value
        self.gps_noise = self.get_parameter('gps_noise_level').value
        self.temp_noise = self.get_parameter('temp_noise_level').value
        self.pressure_noise = self.get_parameter('pressure_noise_level').value
        
        self.init_lat = self.get_parameter('initial_latitude').value
        self.init_lon = self.get_parameter('initial_longitude').value
        self.init_alt = self.get_parameter('initial_altitude').value
        self.init_temp = self.get_parameter('initial_temperature').value
        
        self.enable_motion = self.get_parameter('enable_motion').value
        self.motion_speed = self.get_parameter('motion_speed').value

    def _initialize_sensor_states(self):
        """Initialize internal state variables for all sensors."""
        # IMU state
        self.imu_seq = 0
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 9.81]  # Gravity on Z-axis
        
        # GPS state
        self.gps_seq = 0
        self.current_lat = self.init_lat
        self.current_lon = self.init_lon
        self.current_alt = self.init_alt
        
        # Temperature state
        self.temp_seq = 0
        self.current_temp = self.init_temp
        
        # Barometer state
        self.baro_seq = 0
        self.current_pressure = self._altitude_to_pressure(self.init_alt)
        
        # Time tracking for motion simulation
        self.start_time = self.get_clock().now()

    def _setup_qos_profiles(self):
        """Configure Quality of Service profiles for different data types."""
        # High-frequency IMU data - use BEST_EFFORT for lower latency
        self.imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Low-frequency reliable data - use RELIABLE
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _create_publishers(self):
        """Create publishers for all sensor topics."""
        self.imu_publisher = self.create_publisher(
            Imu, '/imu/data', self.imu_qos)
        
        self.gps_publisher = self.create_publisher(
            NavSatFix, '/gps/fix', self.reliable_qos)
        
        self.temp_publisher = self.create_publisher(
            Temperature, '/temperature/data', self.reliable_qos)
        
        self.baro_publisher = self.create_publisher(
            FluidPressure, '/barometer/data', self.reliable_qos)

    def _create_timers(self):
        """Create timers for periodic sensor data publication."""
        # IMU: 10 Hz (100ms period)
        self.imu_timer = self.create_timer(0.1, self.publish_imu_data)
        
        # GPS: 1 Hz (1000ms period)
        self.gps_timer = self.create_timer(1.0, self.publish_gps_data)
        
        # Temperature: 2 Hz (500ms period)
        self.temp_timer = self.create_timer(0.5, self.publish_temperature_data)
        
        # Barometer: 2 Hz (500ms period)
        self.baro_timer = self.create_timer(0.5, self.publish_barometer_data)

    def _log_configuration(self):
        """Log the current configuration for debugging."""
        self.get_logger().info(f'IMU Noise Level: {self.imu_noise}')
        self.get_logger().info(f'GPS Noise Level: {self.gps_noise}')
        self.get_logger().info(f'Temperature Noise: {self.temp_noise}°C')
        self.get_logger().info(f'Pressure Noise: {self.pressure_noise} Pa')
        self.get_logger().info(
            f'Initial Position: ({self.init_lat}, {self.init_lon}, {self.init_alt}m)')

    def _altitude_to_pressure(self, altitude):
        """
        Convert altitude to atmospheric pressure using barometric formula.
        
        Args:
            altitude (float): Altitude in meters above sea level
            
        Returns:
            float: Atmospheric pressure in Pascals
        """
        # Standard atmospheric pressure at sea level (Pa)
        P0 = 101325.0
        # Temperature lapse rate (K/m)
        L = 0.0065
        # Standard temperature at sea level (K)
        T0 = 288.15
        # Universal gas constant (J/(mol·K))
        R = 8.31447
        # Gravitational acceleration (m/s²)
        g = 9.80665
        # Molar mass of Earth's air (kg/mol)
        M = 0.0289644
        
        # Barometric formula
        pressure = P0 * math.pow((1 - (L * altitude) / T0), (g * M) / (R * L))
        return pressure

    def _add_gaussian_noise(self, value, noise_level):
        """
        Add Gaussian (normal) noise to a sensor reading.
        
        Args:
            value (float): Original sensor value
            noise_level (float): Standard deviation of noise
            
        Returns:
            float: Value with added noise
        """
        return value + np.random.normal(0, noise_level)

    def _simulate_motion(self):
        """
        Simulate realistic robot motion for GPS and IMU.
        
        Creates circular motion pattern with varying altitude.
        """
        if not self.enable_motion:
            return
        
        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Circular motion parameters
        radius = 0.0001  # Approximately 11 meters
        angular_freq = 0.1 * self.motion_speed  # rad/s
        
        # Update position (circular motion)
        self.current_lat = self.init_lat + radius * math.cos(angular_freq * elapsed)
        self.current_lon = self.init_lon + radius * math.sin(angular_freq * elapsed)
        
        # Vary altitude sinusoidally
        self.current_alt = self.init_alt + 10 * math.sin(0.05 * elapsed)
        
        # Update IMU based on motion
        # Angular velocity (turning)
        self.angular_velocity[2] = angular_freq
        
        # Linear acceleration (centripetal)
        speed = radius * 111139 * angular_freq  # Convert to m/s
        centripetal = speed * speed / (radius * 111139)
        self.linear_acceleration[0] = centripetal * math.cos(angular_freq * elapsed)
        self.linear_acceleration[1] = centripetal * math.sin(angular_freq * elapsed)

    def publish_imu_data(self):
        """Publish IMU sensor data at 10 Hz."""
        self._simulate_motion()
        
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Angular velocity (rad/s) with noise
        msg.angular_velocity.x = self._add_gaussian_noise(
            self.angular_velocity[0], self.imu_noise)
        msg.angular_velocity.y = self._add_gaussian_noise(
            self.angular_velocity[1], self.imu_noise)
        msg.angular_velocity.z = self._add_gaussian_noise(
            self.angular_velocity[2], self.imu_noise)
        
        # Linear acceleration (m/s²) with noise
        msg.linear_acceleration.x = self._add_gaussian_noise(
            self.linear_acceleration[0], self.imu_noise)
        msg.linear_acceleration.y = self._add_gaussian_noise(
            self.linear_acceleration[1], self.imu_noise)
        msg.linear_acceleration.z = self._add_gaussian_noise(
            self.linear_acceleration[2], self.imu_noise)
        
        # Orientation (quaternion) - simplified, identity orientation
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        # Covariance matrices (diagonal)
        msg.angular_velocity_covariance[0] = self.imu_noise ** 2
        msg.angular_velocity_covariance[4] = self.imu_noise ** 2
        msg.angular_velocity_covariance[8] = self.imu_noise ** 2
        
        msg.linear_acceleration_covariance[0] = self.imu_noise ** 2
        msg.linear_acceleration_covariance[4] = self.imu_noise ** 2
        msg.linear_acceleration_covariance[8] = self.imu_noise ** 2
        
        self.imu_publisher.publish(msg)
        self.imu_seq += 1

    def publish_gps_data(self):
        """Publish GPS sensor data at 1 Hz."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        
        # Position with noise
        msg.latitude = self._add_gaussian_noise(self.current_lat, self.gps_noise)
        msg.longitude = self._add_gaussian_noise(self.current_lon, self.gps_noise)
        msg.altitude = self._add_gaussian_noise(self.current_alt, self.gps_noise * 10)
        
        # GPS status
        msg.status.status = 0  # STATUS_FIX (valid fix)
        msg.status.service = 1  # SERVICE_GPS
        
        # Position covariance
        msg.position_covariance[0] = self.gps_noise ** 2
        msg.position_covariance[4] = self.gps_noise ** 2
        msg.position_covariance[8] = (self.gps_noise * 10) ** 2
        msg.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(msg)
        self.gps_seq += 1
        
        if self.gps_seq % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f'GPS: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, '
                f'Alt={msg.altitude:.2f}m')

    def publish_temperature_data(self):
        """Publish temperature sensor data at 2 Hz."""
        msg = Temperature()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temp_sensor_link'
        
        # Simulate temperature variation (diurnal cycle)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        temp_variation = 5.0 * math.sin(0.01 * elapsed)  # ±5°C variation
        
        self.current_temp = self.init_temp + temp_variation
        msg.temperature = self._add_gaussian_noise(self.current_temp, self.temp_noise)
        msg.variance = self.temp_noise ** 2
        
        self.temp_publisher.publish(msg)
        self.temp_seq += 1

    def publish_barometer_data(self):
        """Publish barometer/pressure sensor data at 2 Hz."""
        msg = FluidPressure()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'barometer_link'
        
        # Calculate pressure from current altitude
        self.current_pressure = self._altitude_to_pressure(self.current_alt)
        
        # Add noise
        msg.fluid_pressure = self._add_gaussian_noise(
            self.current_pressure, self.pressure_noise)
        msg.variance = self.pressure_noise ** 2
        
        self.baro_publisher.publish(msg)
        self.baro_seq += 1


def main(args=None):
    """Main function to initialize and run the sensor publisher node."""
    rclpy.init(args=args)
    
    try:
        sensor_publisher = SensorPublisher()
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in sensor publisher: {e}')
    finally:
        if rclpy.ok():
            sensor_publisher.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()