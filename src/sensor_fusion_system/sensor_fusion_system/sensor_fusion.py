import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
import numpy as np
from collections import deque
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float64

class SensorFusionNode(Node):
    """
    Subscribes to sensor data, performs filtering (moving average),
    calculates altitude from barometer data, and publishes fused/derived outputs.
    """
    
    # Constants for altitude calculation
    SEA_LEVEL_PRESSURE_PA = 101325.0  # (Pa)
    
    def __init__(self):
        # اسم النود في الطرفية سيكون 'sensor_fusion_node'
        super().__init__('sensor_fusion_node') 

        # الإعلان عن الإعدادات واستلامها
        self.declare_parameter('temp_filter_window_size', 10)
        self.declare_parameter('altitude_filter_window_size', 5)
        temp_window_size = self.get_parameter('temp_filter_window_size').value
        alt_window_size = self.get_parameter('altitude_filter_window_size').value
        
        # مخازن بيانات مؤقتة للفلترة
        self.temp_buffer = deque(maxlen=temp_window_size)
        self.altitude_buffer = deque(maxlen=alt_window_size)

        # المشتركون (Subscribers)
        qos_profile = SensorDataQoS()
        self.create_subscription(
            Temperature, '/sensors/temperature', self.temp_callback, qos_profile)
        self.create_subscription(
            FluidPressure, '/sensors/barometer', self.pressure_callback, qos_profile)

        # الناشرون (Publishers)
        # هذه هي المواضيع التي يستمع إليها 'visualization.py'
        self.fused_temp_pub = self.create_publisher(
            Float64, '/fused/temperature', 10)
        self.fused_altitude_pub = self.create_publisher(
            Float64, '/fused/altitude', 10)

        self.get_logger().info("Sensor Fusion Node started (Visualizer-Compatible Version).")
        self.get_logger().info(f"Temp filter window: {temp_window_size}, Altitude filter window: {alt_window_size}")

    def temp_callback(self, msg: Temperature):
        """يستقبل الحرارة، يفلترها، وينشرها"""
        self.temp_buffer.append(msg.temperature)
        
        if self.temp_buffer:
            fused_temp = np.mean(self.temp_buffer)
            pub_msg = Float64()
            pub_msg.data = fused_temp
            self.fused_temp_pub.publish(pub_msg)
            
    def pressure_callback(self, msg: FluidPressure):
        """يستقبل الضغط، يحسب الارتفاع، يفلتره، وينشره"""
        try:
            pressure_pa = msg.fluid_pressure
            # حساب الارتفاع
            pressure_ratio = pressure_pa / self.SEA_LEVEL_PRESSURE_PA
            raw_altitude = 44330.0 * (1.0 - (pressure_ratio ** 0.190284))
            
            # فلترة الارتفاع
            self.altitude_buffer.append(raw_altitude)
            
            if self.altitude_buffer:
                fused_altitude = np.mean(self.altitude_buffer)
                pub_msg = Float64()
                pub_msg.data = fused_altitude
                self.fused_altitude_pub.publish(pub_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error calculating altitude: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SensorFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sensor Fusion shutting down.")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()