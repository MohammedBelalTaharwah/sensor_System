#!/usr/bin/env python3
"""
Sensor Fusion Node for Multi-Sensor Integration

Combines data from multiple sensors (IMU, GPS, Barometer, Temperature)
to produce a fused state estimate with improved accuracy.

Features:
- Complementary filter for IMU/GPS fusion
- Kalman filter for altitude estimation
- Moving average filter for noise reduction
- Temperature-compensated barometer readings

Author: ROS2 Jazzy Implementation
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
from collections import deque
import math


class MovingAverageFilter:
    """Simple moving average filter for sensor data smoothing."""
    
    def __init__(self, window_size=5):
        """
        Initialize moving average filter.
        
        Args:
            window_size (int): Number of samples to average
        """
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
    
    def update(self, value):
        """
        Add new value and return filtered result.
        
        Args:
            value (float): New sensor reading
            
        Returns:
            float: Filtered value (moving average)
        """
        self.data.append(value)
        return np.mean(self.data)
    
    def reset(self):
        """Clear the filter buffer."""
        self.data.clear()


class KalmanFilter:
    """
    Simple 1D Kalman filter for altitude estimation.
    
    Fuses barometer and GPS altitude measurements.
    """
    
    def __init__(self, process_variance=1e-5, measurement_variance=1e-2):
        """
        Initialize Kalman filter.
        
        Args:
            process_variance (float): Process noise variance
            measurement_variance (float): Measurement noise variance
        """
        self.x = 0.0  # State estimate (altitude)
        self.P = 1.0  # Error covariance
        self.Q = process_variance  # Process noise covariance
        self.R = measurement_variance  # Measurement noise covariance
        
    def predict(self):
        """Prediction step: propagate state forward."""
        # For stationary altitude, prediction doesn't change estimate
        self.P += self.Q
        
    def update(self, measurement):
        """
        Update step: incorporate new measurement.
        
        Args:
            measurement (float): New altitude measurement
            
        Returns:
            float: Updated state estimate
        """
        # Kalman gain
        K = self.P / (self.P + self.R)
        
        # Update estimate with measurement
        self.x = self.x + K * (measurement - self.x)
        
        # Update error covariance
        self.P = (1 - K) * self.P
        
        return self.x


class SensorFusion(Node):
    """
    Fuses multiple sensor inputs for robust state estimation.
    
    Combines:
    - IMU: Orientation and acceleration
    - GPS: Position
    - Barometer: Altitude
    - Temperature: Environmental compensation
    """

    def __init__(self):
        """Initialize the sensor fusion node."""
        super().__init__('sensor_fusion')
        
        # Declare parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize filters
        self._initialize_filters()
        
        # Initialize state
        self._initialize_state()
        
        # Setup QoS
        self._setup_qos_profiles()
        
        # Create subscribers
        self._create_subscribers()
        
        # Create publishers
        self.fused_pub = self.create_publisher(
            PoseStamped, '/fused_data', self.reliable_qos)
        
        # Create fusion timer
        self.fusion_timer = self.create_timer(
            self.fusion_rate, self.publish_fused_data)
        
        self.get_logger().info('Sensor Fusion Node initialized successfully')
        self.get_logger().info(f'Fusion rate: {1.0/self.fusion_rate:.1f} Hz')

    def _declare_parameters(self):
        """Declare configurable parameters."""
        self.declare_parameter('fusion_rate', 0.1)  # 10 Hz
        self.declare_parameter('altitude_filter_window', 5)
        self.declare_parameter('gps_weight', 0.3)
        self.declare_parameter('baro_weight', 0.7)
        self.declare_parameter('enable_kalman', True)
        self.declare_parameter('sea_level_pressure', 101325.0)

    def _get_parameters(self):
        """Retrieve parameter values."""
        self.fusion_rate = self.get_parameter('fusion_rate').value
        self.alt_window = self.get_parameter('altitude_filter_window').value
        self.gps_weight = self.get_parameter('gps_weight').value
        self.baro_weight = self.get_parameter('baro_weight').value
        self.use_kalman = self.get_parameter('enable_kalman').value
        self.sea_level_p = self.get_parameter('sea_level_pressure').value

    def _initialize_filters(self):
        """Initialize filtering algorithms."""
        # Moving average filters
        self.alt_filter = MovingAverageFilter(self.alt_window)
        self.lat_filter = MovingAverageFilter(5)
        self.lon_filter = MovingAverageFilter(5)
        
        # Kalman filter for altitude
        self.altitude_kalman = KalmanFilter(
            process_variance=1e-5,
            measurement_variance=1e-2
        )

    def _initialize_state(self):
        """Initialize internal state variables."""
        # Latest sensor data
        self.latest_imu = None
        self.latest_gps = None
        self.latest_temp = None
        self.latest_baro = None
        
        # Fused state
        self.fused_position = Point()
        self.fused_orientation = Quaternion()
        self.fused_orientation.w = 1.0  # Identity quaternion
        
        # Altitude estimates
        self.gps_altitude = 0.0
        self.baro_altitude = 0.0
        self.fused_altitude = 0.0
        
        # Temperature for compensation
        self.current_temperature = 15.0  # Default 15°C
        
        # Data availability flags
        self.has_imu = False
        self.has_gps = False
        self.has_baro = False
        self.has_temp = False

    def _setup_qos_profiles(self):
        """Configure QoS profiles."""
        self.imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _create_subscribers(self):
        """Create subscribers for all sensor topics."""
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, self.imu_qos)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, self.reliable_qos)
        
        self.temp_sub = self.create_subscription(
            Temperature, '/temperature/data', self.temp_callback, self.reliable_qos)
        
        self.baro_sub = self.create_subscription(
            FluidPressure, '/barometer/data', self.baro_callback, self.reliable_qos)

    def _pressure_to_altitude(self, pressure, temperature=15.0):
        """
        Convert pressure to altitude using hypsometric formula.
        
        Args:
            pressure (float): Atmospheric pressure in Pascals
            temperature (float): Temperature in Celsius
            
        Returns:
            float: Altitude in meters
        """
        # Convert temperature to Kelvin
        T = temperature + 273.15
        
        # Hypsometric formula
        altitude = ((T / 0.0065) * 
                   (1 - math.pow(pressure / self.sea_level_p, 0.190284)))
        
        return altitude

    def imu_callback(self, msg):
        """
        Process IMU data.
        
        Args:
            msg (Imu): IMU sensor message
        """
        self.latest_imu = msg
        self.has_imu = True
        
        # Extract orientation (if available)
        self.fused_orientation = msg.orientation

    def gps_callback(self, msg):
        """
        Process GPS data.
        
        Args:
            msg (NavSatFix): GPS sensor message
        """
        self.latest_gps = msg
        self.has_gps = True
        
        # Apply moving average filter to position
        filtered_lat = self.lat_filter.update(msg.latitude)
        filtered_lon = self.lon_filter.update(msg.longitude)
        
        # Update position
        self.fused_position.x = filtered_lat
        self.fused_position.y = filtered_lon
        
        # Store GPS altitude
        self.gps_altitude = msg.altitude

    def temp_callback(self, msg):
        """
        Process temperature data for barometer compensation.
        
        Args:
            msg (Temperature): Temperature sensor message
        """
        self.latest_temp = msg
        self.has_temp = True
        self.current_temperature = msg.temperature

    def baro_callback(self, msg):
        """
        Process barometer data and calculate altitude.
        
        Args:
            msg (FluidPressure): Pressure sensor message
        """
        self.latest_baro = msg
        self.has_baro = True
        
        # Convert pressure to altitude with temperature compensation
        self.baro_altitude = self._pressure_to_altitude(
            msg.fluid_pressure, 
            self.current_temperature
        )

    def _fuse_altitude(self):
        """
        Fuse GPS and barometer altitude estimates.
        
        Uses weighted average or Kalman filter depending on configuration.
        
        Returns:
            float: Fused altitude estimate in meters
        """
        if not (self.has_gps and self.has_baro):
            # Use whichever is available
            if self.has_gps:
                return self.gps_altitude
            elif self.has_baro:
                return self.baro_altitude
            else:
                return 0.0
        
        if self.use_kalman:
            # Use Kalman filter
            self.altitude_kalman.predict()
            
            # Weight measurements based on their reliability
            combined_measurement = (self.gps_weight * self.gps_altitude + 
                                   self.baro_weight * self.baro_altitude)
            
            fused_alt = self.altitude_kalman.update(combined_measurement)
        else:
            # Simple weighted average
            fused_alt = (self.gps_weight * self.gps_altitude + 
                        self.baro_weight * self.baro_altitude)
        
        # Apply moving average filter
        fused_alt = self.alt_filter.update(fused_alt)
        
        return fused_alt

    def publish_fused_data(self):
        """Publish fused sensor data."""
        # Calculate fused altitude
        self.fused_altitude = self._fuse_altitude()
        self.fused_position.z = self.fused_altitude
        
        # Create and populate message
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position
        msg.pose.position = self.fused_position
        
        # Set orientation (from IMU if available)
        if self.has_imu:
            msg.pose.orientation = self.fused_orientation
        else:
            msg.pose.orientation.w = 1.0  # Identity
        
        # Publish
        self.fused_pub.publish(msg)
        
        # Log periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(
                f'Fused State: Pos=({self.fused_position.x:.6f}, '
                f'{self.fused_position.y:.6f}, {self.fused_position.z:.2f}m) | '
                f'GPS_alt={self.gps_altitude:.2f}m, Baro_alt={self.baro_altitude:.2f}m',
                throttle_duration_sec=5.0
            )


def main(args=None):
    """Main function to initialize and run the sensor fusion node."""
    rclpy.init(args=args)
    
    try:
        sensor_fusion = SensorFusion()
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in sensor fusion: {e}')
    finally:
        if rclpy.ok():
            sensor_fusion.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()