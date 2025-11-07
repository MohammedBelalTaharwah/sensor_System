#!/usr/bin/env python3
"""
Alert System Node
Monitors all sensors and detects anomalies
Publishes alerts for out-of-range values
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from std_msgs.msg import String
import math


class AlertSystem(Node):
    """Node for monitoring sensors and detecting anomalies"""
    
    # Define sensor thresholds
    THRESHOLDS = {
        'imu_accel_max': 20.0,  # m/s²
        'imu_gyro_max': 5.0,    # rad/s
        'temp_min': -40.0,      # °C
        'temp_max': 85.0,       # °C
        'pressure_min': 30000.0,  # Pa (very low altitude)
        'pressure_max': 120000.0, # Pa (below sea level)
        'altitude_min': -500.0,   # meters
        'altitude_max': 10000.0,  # meters
    }
    
    def __init__(self):
        super().__init__('alert_system')
        
        # Declare parameters
        self.declare_parameter('alert_cooldown', 5.0)  # seconds between same alerts
        
        # Alert tracking
        self.last_alerts = {}
        self.alert_counts = {}
        
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
        
        # Create alert publisher
        self.alert_pub = self.create_publisher(String, '/alerts', 10)
        
        # Status timer (every 10 seconds)
        self.status_timer = self.create_timer(10.0, self.print_status)
        
        self.get_logger().info('Alert System initialized and monitoring...')
        
    def should_publish_alert(self, alert_type):
        """Check if enough time has passed since last alert of this type"""
        cooldown = self.get_parameter('alert_cooldown').value
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if alert_type not in self.last_alerts:
            self.last_alerts[alert_type] = current_time
            self.alert_counts[alert_type] = 1
            return True
        
        time_diff = current_time - self.last_alerts[alert_type]
        
        if time_diff > cooldown:
            self.last_alerts[alert_type] = current_time
            self.alert_counts[alert_type] = self.alert_counts.get(alert_type, 0) + 1
            return True
        
        return False
    
    def publish_alert(self, severity, sensor, message):
        """Publish an alert message"""
        alert_type = f"{sensor}_{message}"
        
        if self.should_publish_alert(alert_type):
            alert_msg = String()
            alert_msg.data = f"[{severity}] {sensor}: {message}"
            self.alert_pub.publish(alert_msg)
            
            # Log based on severity
            if severity == "CRITICAL":
                self.get_logger().error(alert_msg.data)
            elif severity == "WARNING":
                self.get_logger().warn(alert_msg.data)
            else:
                self.get_logger().info(alert_msg.data)
    
    def imu_callback(self, msg):
        """Monitor IMU data for anomalies"""
        try:
            # Check acceleration magnitude
            accel_mag = math.sqrt(
                msg.linear_acceleration.x**2 + 
                msg.linear_acceleration.y**2 + 
                msg.linear_acceleration.z**2
            )
            
            if accel_mag > self.THRESHOLDS['imu_accel_max']:
                self.publish_alert(
                    "WARNING",
                    "IMU",
                    f"High acceleration detected: {accel_mag:.2f} m/s²"
                )
            
            # Check gyroscope magnitude
            gyro_mag = math.sqrt(
                msg.angular_velocity.x**2 + 
                msg.angular_velocity.y**2 + 
                msg.angular_velocity.z**2
            )
            
            if gyro_mag > self.THRESHOLDS['imu_gyro_max']:
                self.publish_alert(
                    "WARNING",
                    "IMU",
                    f"High angular velocity detected: {gyro_mag:.2f} rad/s"
                )
            
            # Check for NaN values
            if (math.isnan(msg.linear_acceleration.x) or 
                math.isnan(msg.angular_velocity.x)):
                self.publish_alert(
                    "CRITICAL",
                    "IMU",
                    "Invalid sensor reading (NaN detected)"
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing IMU alert: {e}')
    
    def gps_callback(self, msg):
        """Monitor GPS data for anomalies"""
        try:
            # Check altitude range
            if msg.altitude < self.THRESHOLDS['altitude_min']:
                self.publish_alert(
                    "WARNING",
                    "GPS",
                    f"Altitude too low: {msg.altitude:.2f} m"
                )
            elif msg.altitude > self.THRESHOLDS['altitude_max']:
                self.publish_alert(
                    "WARNING",
                    "GPS",
                    f"Altitude too high: {msg.altitude:.2f} m"
                )
            
            # Check for invalid coordinates
            if abs(msg.latitude) > 90:
                self.publish_alert(
                    "CRITICAL",
                    "GPS",
                    f"Invalid latitude: {msg.latitude}°"
                )
            
            if abs(msg.longitude) > 180:
                self.publish_alert(
                    "CRITICAL",
                    "GPS",
                    f"Invalid longitude: {msg.longitude}°"
                )
            
            # Check position covariance if available
            if hasattr(msg, 'position_covariance'):
                # Diagonal elements represent variance in x, y, z
                if msg.position_covariance[0] > 100.0:  # High uncertainty
                    self.publish_alert(
                        "INFO",
                        "GPS",
                        "Poor GPS accuracy detected"
                    )
                    
        except Exception as e:
            self.get_logger().error(f'Error processing GPS alert: {e}')
    
    def temp_callback(self, msg):
        """Monitor Temperature data for anomalies"""
        try:
            temp_celsius = msg.temperature
            
            if temp_celsius < self.THRESHOLDS['temp_min']:
                self.publish_alert(
                    "WARNING",
                    "Temperature",
                    f"Temperature too low: {temp_celsius:.2f} °C"
                )
            elif temp_celsius > self.THRESHOLDS['temp_max']:
                self.publish_alert(
                    "WARNING",
                    "Temperature",
                    f"Temperature too high: {temp_celsius:.2f} °C"
                )
            
            # Check for extreme changes (if we had previous value)
            if not hasattr(self, 'last_temp'):
                self.last_temp = temp_celsius
                return
            
            temp_change = abs(temp_celsius - self.last_temp)
            if temp_change > 10.0:  # More than 10°C change
                self.publish_alert(
                    "WARNING",
                    "Temperature",
                    f"Rapid temperature change: {temp_change:.2f} °C"
                )
            
            self.last_temp = temp_celsius
            
        except Exception as e:
            self.get_logger().error(f'Error processing Temperature alert: {e}')
    
    def baro_callback(self, msg):
        """Monitor Barometer data for anomalies"""
        try:
            pressure = msg.fluid_pressure
            
            if pressure < self.THRESHOLDS['pressure_min']:
                self.publish_alert(
                    "WARNING",
                    "Barometer",
                    f"Pressure too low: {pressure:.2f} Pa"
                )
            elif pressure > self.THRESHOLDS['pressure_max']:
                self.publish_alert(
                    "WARNING",
                    "Barometer",
                    f"Pressure too high: {pressure:.2f} Pa"
                )
            
            # Check for rapid pressure changes
            if not hasattr(self, 'last_pressure'):
                self.last_pressure = pressure
                return
            
            pressure_change = abs(pressure - self.last_pressure)
            if pressure_change > 1000.0:  # More than 1000 Pa change
                self.publish_alert(
                    "INFO",
                    "Barometer",
                    f"Rapid pressure change: {pressure_change:.2f} Pa"
                )
            
            self.last_pressure = pressure
            
        except Exception as e:
            self.get_logger().error(f'Error processing Barometer alert: {e}')
    
    def print_status(self):
        """Print alert system status"""
        if self.alert_counts:
            self.get_logger().info('--- Alert Summary ---')
            for alert_type, count in self.alert_counts.items():
                self.get_logger().info(f'  {alert_type}: {count} alerts')
        else:
            self.get_logger().info('All sensors operating normally')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        alert_system = AlertSystem()
        rclpy.spin(alert_system)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in alert system: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()