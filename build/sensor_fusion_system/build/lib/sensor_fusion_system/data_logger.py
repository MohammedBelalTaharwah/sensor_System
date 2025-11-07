#!/usr/bin/env python3
"""
Data Logger Node
Subscribes to all sensor topics and logs data with statistics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from std_msgs.msg import Float32
from collections import deque
import statistics
import json
from datetime import datetime


class DataLogger(Node):
    """Node for logging sensor data and calculating statistics"""
    
    def __init__(self):
        super().__init__('data_logger')
        
        # Initialize data storage with limited history
        self.imu_data = deque(maxlen=100)
        self.gps_data = deque(maxlen=50)
        self.temp_data = deque(maxlen=50)
        self.baro_data = deque(maxlen=50)
        
        # Create subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu',
            self.imu_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/sensors/gps',
            self.gps_callback,
            10
        )
        
        self.temp_sub = self.create_subscription(
            Temperature,
            '/sensors/temperature',
            self.temp_callback,
            10
        )
        
        self.baro_sub = self.create_subscription(
            FluidPressure,
            '/sensors/barometer',
            self.baro_callback,
            10
        )
        
        # Statistics timer (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        # Data logging timer (every 1 second)
        self.log_timer = self.create_timer(1.0, self.log_to_file)
        
        # Initialize log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f'sensor_log_{timestamp}.json'
        
        self.get_logger().info(f'Data Logger initialized. Logging to: {self.log_filename}')
        
    def imu_callback(self, msg):
        """Process IMU data"""
        try:
            data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'accel_x': msg.linear_acceleration.x,
                'accel_y': msg.linear_acceleration.y,
                'accel_z': msg.linear_acceleration.z,
                'gyro_x': msg.angular_velocity.x,
                'gyro_y': msg.angular_velocity.y,
                'gyro_z': msg.angular_velocity.z
            }
            self.imu_data.append(data)
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')
    
    def gps_callback(self, msg):
        """Process GPS data"""
        try:
            data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude
            }
            self.gps_data.append(data)
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {e}')
    
    def temp_callback(self, msg):
        """Process Temperature data"""
        try:
            data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'temperature': msg.temperature
            }
            self.temp_data.append(data)
        except Exception as e:
            self.get_logger().error(f'Error processing Temperature data: {e}')
    
    def baro_callback(self, msg):
        """Process Barometer data"""
        try:
            data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'pressure': msg.fluid_pressure
            }
            self.baro_data.append(data)
        except Exception as e:
            self.get_logger().error(f'Error processing Barometer data: {e}')
    
    def calculate_statistics(self, data_list, field):
        """Calculate min, max, average for a specific field"""
        if not data_list:
            return None, None, None
        
        try:
            values = [d[field] for d in data_list]
            return min(values), max(values), statistics.mean(values)
        except Exception as e:
            self.get_logger().error(f'Error calculating statistics: {e}')
            return None, None, None
    
    def print_statistics(self):
        """Print statistics summary every 5 seconds"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('SENSOR STATISTICS SUMMARY')
        self.get_logger().info('=' * 60)
        
        # IMU Statistics
        if self.imu_data:
            self.get_logger().info(f'IMU Data Points: {len(self.imu_data)}')
            
            for axis, field in [('X', 'accel_x'), ('Y', 'accel_y'), ('Z', 'accel_z')]:
                min_val, max_val, avg_val = self.calculate_statistics(self.imu_data, field)
                if min_val is not None:
                    self.get_logger().info(
                        f'  Accel {axis}: Min={min_val:.3f}, Max={max_val:.3f}, Avg={avg_val:.3f} m/s²'
                    )
        
        # GPS Statistics
        if self.gps_data:
            self.get_logger().info(f'GPS Data Points: {len(self.gps_data)}')
            
            lat_min, lat_max, lat_avg = self.calculate_statistics(self.gps_data, 'latitude')
            lon_min, lon_max, lon_avg = self.calculate_statistics(self.gps_data, 'longitude')
            alt_min, alt_max, alt_avg = self.calculate_statistics(self.gps_data, 'altitude')
            
            if lat_min is not None:
                self.get_logger().info(f'  Latitude: Min={lat_min:.6f}, Max={lat_max:.6f}, Avg={lat_avg:.6f}°')
                self.get_logger().info(f'  Longitude: Min={lon_min:.6f}, Max={lon_max:.6f}, Avg={lon_avg:.6f}°')
                self.get_logger().info(f'  Altitude: Min={alt_min:.2f}, Max={alt_max:.2f}, Avg={alt_avg:.2f} m')
        
        # Temperature Statistics
        if self.temp_data:
            self.get_logger().info(f'Temperature Data Points: {len(self.temp_data)}')
            
            temp_min, temp_max, temp_avg = self.calculate_statistics(self.temp_data, 'temperature')
            if temp_min is not None:
                self.get_logger().info(
                    f'  Temperature: Min={temp_min:.2f}, Max={temp_max:.2f}, Avg={temp_avg:.2f} °C'
                )
        
        # Barometer Statistics
        if self.baro_data:
            self.get_logger().info(f'Barometer Data Points: {len(self.baro_data)}')
            
            press_min, press_max, press_avg = self.calculate_statistics(self.baro_data, 'pressure')
            if press_min is not None:
                self.get_logger().info(
                    f'  Pressure: Min={press_min:.2f}, Max={press_max:.2f}, Avg={press_avg:.2f} Pa'
                )
        
        self.get_logger().info('=' * 60)
    
    def log_to_file(self):
        """Log current data to file"""
        try:
            log_entry = {
                'timestamp': datetime.now().isoformat(),
                'imu_count': len(self.imu_data),
                'gps_count': len(self.gps_data),
                'temp_count': len(self.temp_data),
                'baro_count': len(self.baro_data),
                'latest_data': {
                    'imu': list(self.imu_data)[-1] if self.imu_data else None,
                    'gps': list(self.gps_data)[-1] if self.gps_data else None,
                    'temperature': list(self.temp_data)[-1] if self.temp_data else None,
                    'barometer': list(self.baro_data)[-1] if self.baro_data else None
                }
            }
            
            with open(self.log_filename, 'a') as f:
                json.dump(log_entry, f)
                f.write('\n')
                
        except Exception as e:
            self.get_logger().error(f'Error writing to log file: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        data_logger = DataLogger()
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in data logger: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()