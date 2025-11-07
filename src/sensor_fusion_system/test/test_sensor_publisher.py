#!/usr/bin/env python3
"""
Unit tests for Sensor Publisher Node
Tests sensor data generation and publishing
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
import time


class TestSensorPublisher(unittest.TestCase):
    """Test cases for sensor publisher"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures"""
        self.node = Node('test_sensor_publisher')
        self.received_msgs = {
            'imu': [],
            'gps': [],
            'temp': [],
            'baro': []
        }
        
        # Create subscribers
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/sensors/imu',
            lambda msg: self.received_msgs['imu'].append(msg),
            10
        )
        
        self.gps_sub = self.node.create_subscription(
            NavSatFix,
            '/sensors/gps',
            lambda msg: self.received_msgs['gps'].append(msg),
            10
        )
        
        self.temp_sub = self.node.create_subscription(
            Temperature,
            '/sensors/temperature',
            lambda msg: self.received_msgs['temp'].append(msg),
            10
        )
        
        self.baro_sub = self.node.create_subscription(
            FluidPressure,
            '/sensors/barometer',
            lambda msg: self.received_msgs['baro'].append(msg),
            10
        )
    
    def tearDown(self):
        """Clean up test fixtures"""
        self.node.destroy_node()
    
    def spin_for_duration(self, duration=2.0):
        """Spin node for specified duration"""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def test_imu_publishing(self):
        """Test that IMU messages are being published"""
        self.spin_for_duration(2.0)
        
        # Should receive multiple IMU messages (10 Hz)
        self.assertGreater(len(self.received_msgs['imu']), 5,
                          "Should receive multiple IMU messages")
        
        # Check message structure
        if self.received_msgs['imu']:
            msg = self.received_msgs['imu'][0]
            self.assertIsInstance(msg, Imu)
            self.assertTrue(hasattr(msg, 'linear_acceleration'))
            self.assertTrue(hasattr(msg, 'angular_velocity'))
    
    def test_imu_data_validity(self):
        """Test that IMU data is within reasonable ranges"""
        self.spin_for_duration(2.0)
        
        for msg in self.received_msgs['imu']:
            # Check acceleration magnitude (should be around 9.81 m/s² ± noise)
            accel_z = msg.linear_acceleration.z
            self.assertGreater(accel_z, 5.0, "Z acceleration too low")
            self.assertLess(accel_z, 15.0, "Z acceleration too high")
            
            # Check angular velocity (should be small in stationary case)
            gyro_x = abs(msg.angular_velocity.x)
            self.assertLess(gyro_x, 2.0, "Angular velocity too high")
    
    def test_gps_publishing(self):
        """Test that GPS messages are being published"""
        self.spin_for_duration(3.0)
        
        # Should receive at least one GPS message (1 Hz)
        self.assertGreater(len(self.received_msgs['gps']), 0,
                          "Should receive GPS messages")
        
        # Check message structure
        if self.received_msgs['gps']:
            msg = self.received_msgs['gps'][0]
            self.assertIsInstance(msg, NavSatFix)
            self.assertTrue(hasattr(msg, 'latitude'))
            self.assertTrue(hasattr(msg, 'longitude'))
            self.assertTrue(hasattr(msg, 'altitude'))
    
    def test_gps_data_validity(self):
        """Test that GPS data is within valid ranges"""
        self.spin_for_duration(3.0)
        
        for msg in self.received_msgs['gps']:
            # Check latitude range (-90 to 90)
            self.assertGreaterEqual(msg.latitude, -90.0)
            self.assertLessEqual(msg.latitude, 90.0)
            
            # Check longitude range (-180 to 180)
            self.assertGreaterEqual(msg.longitude, -180.0)
            self.assertLessEqual(msg.longitude, 180.0)
            
            # Check altitude (reasonable range)
            self.assertGreater(msg.altitude, -1000.0)
            self.assertLess(msg.altitude, 10000.0)
    
    def test_temperature_publishing(self):
        """Test that Temperature messages are being published"""
        self.spin_for_duration(2.0)
        
        # Should receive at least one Temperature message (2 Hz)
        self.assertGreater(len(self.received_msgs['temp']), 0,
                          "Should receive Temperature messages")
        
        # Check message structure
        if self.received_msgs['temp']:
            msg = self.received_msgs['temp'][0]
            self.assertIsInstance(msg, Temperature)
            self.assertTrue(hasattr(msg, 'temperature'))
    
    def test_temperature_data_validity(self):
        """Test that Temperature data is within reasonable ranges"""
        self.spin_for_duration(2.0)
        
        for msg in self.received_msgs['temp']:
            # Check temperature range (-50 to 100 °C)
            self.assertGreater(msg.temperature, -50.0)
            self.assertLess(msg.temperature, 100.0)
    
    def test_barometer_publishing(self):
        """Test that Barometer messages are being published"""
        self.spin_for_duration(2.0)
        
        # Should receive at least one Barometer message (2 Hz)
        self.assertGreater(len(self.received_msgs['baro']), 0,
                          "Should receive Barometer messages")
        
        # Check message structure
        if self.received_msgs['baro']:
            msg = self.received_msgs['baro'][0]
            self.assertIsInstance(msg, FluidPressure)
            self.assertTrue(hasattr(msg, 'fluid_pressure'))
    
    def test_barometer_data_validity(self):
        """Test that Barometer data is within reasonable ranges"""
        self.spin_for_duration(2.0)
        
        for msg in self.received_msgs['baro']:
            # Check pressure range (50000 to 110000 Pa)
            self.assertGreater(msg.fluid_pressure, 50000.0)
            self.assertLess(msg.fluid_pressure, 110000.0)
    
    def test_publishing_rates(self):
        """Test that sensors publish at expected rates"""
        # Clear previous messages
        self.received_msgs = {
            'imu': [],
            'gps': [],
            'temp': [],
            'baro': []
        }
        
        # Collect data for 5 seconds
        self.spin_for_duration(5.0)
        
        # Check IMU rate (should be ~50 messages in 5 seconds at 10 Hz)
        imu_count = len(self.received_msgs['imu'])
        self.assertGreater(imu_count, 40, f"IMU rate too low: {imu_count} messages")
        self.assertLess(imu_count, 60, f"IMU rate too high: {imu_count} messages")
        
        # Check GPS rate (should be ~5 messages in 5 seconds at 1 Hz)
        gps_count = len(self.received_msgs['gps'])
        self.assertGreater(gps_count, 3, f"GPS rate too low: {gps_count} messages")
        self.assertLess(gps_count, 7, f"GPS rate too high: {gps_count} messages")
        
        # Check Temperature rate (should be ~10 messages in 5 seconds at 2 Hz)
        temp_count = len(self.received_msgs['temp'])
        self.assertGreater(temp_count, 7, f"Temp rate too low: {temp_count} messages")
        self.assertLess(temp_count, 13, f"Temp rate too high: {temp_count} messages")
        
        # Check Barometer rate (should be ~10 messages in 5 seconds at 2 Hz)
        baro_count = len(self.received_msgs['baro'])
        self.assertGreater(baro_count, 7, f"Baro rate too low: {baro_count} messages")
        self.assertLess(baro_count, 13, f"Baro rate too high: {baro_count} messages")


class TestDataConsistency(unittest.TestCase):
    """Test cases for data consistency"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Set up test fixtures"""
        self.node = Node('test_data_consistency')
        self.imu_msgs = []
        
        self.imu_sub = self.node.create_subscription(
            Imu,
            '/sensors/imu',
            lambda msg: self.imu_msgs.append(msg),
            10
        )
    
    def tearDown(self):
        """Clean up test fixtures"""
        self.node.destroy_node()
    
    def test_imu_data_continuity(self):
        """Test that IMU data changes continuously (not stuck)"""
        # Collect some messages
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Check that values are different (sensor is not stuck)
        if len(self.imu_msgs) >= 2:
            first_msg = self.imu_msgs[0]
            last_msg = self.imu_msgs[-1]
            
            # At least one value should be different
            values_changed = (
                first_msg.linear_acceleration.x != last_msg.linear_acceleration.x or
                first_msg.linear_acceleration.y != last_msg.linear_acceleration.y or
                first_msg.angular_velocity.x != last_msg.angular_velocity.x
            )
            
            self.assertTrue(values_changed, "Sensor data appears stuck")


def main():
    """Run tests"""
    unittest.main()


if __name__ == '__main__':
    main()