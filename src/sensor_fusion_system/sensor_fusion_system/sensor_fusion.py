import rclpy
from rclpy.node import Node
# تم حذف 'SensorDataQoS' من هذا السطر
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
        super().__init__('sensor_fusion_node') 

        self.declare_parameter('temp_filter_window_size', 10)
        self.declare_parameter('altitude_filter_window_size', 5)
        temp_window_size = self.get_parameter('temp_filter_window_size').value
        alt_window_size = self.get_parameter('altitude_filter_window_size').value
        
        self.temp_buffer = deque(maxlen=temp_window_size)
        self.altitude_buffer = deque(maxlen=alt_window_size)

        # --- تم التعديل هنا ---
        # المشتركون (Subscribers)
        # تم حذف 'qos_profile = SensorDataQoS()'
        # نستخدم '10' (الإعداد الافتراضي) تماماً مثل 'visualization.py'
        self.create_subscription(
            Temperature, '/sensors/temperature', self.temp_callback, 10)
        self.create_subscription(
            FluidPressure, '/sensors/barometer', self.pressure_callback, 10)
        # --- نهاية التعديل ---

        # الناشرون (Publishers)
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
            pressure_ratio = pressure_pa / self.SEA_LEVEL_PRESSURE_PA
            raw_altitude = 44330.0 * (1.0 - (pressure_ratio ** 0.190284))
            
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