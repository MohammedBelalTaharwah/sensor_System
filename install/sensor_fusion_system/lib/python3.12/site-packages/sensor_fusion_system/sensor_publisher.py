import rclpy
from rclpy.node import Node
import numpy as np
# --- تعديل ---: إضافة 'NavSatStatus'
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure, NavSatStatus 
from geometry_msgs.msg import Quaternion

class SensorPublisherNode(Node):
    """
    Simulates and publishes data from multiple sensors (IMU, GPS, Temp, Barometer)
    with realistic noise.
    """
    def __init__(self):
        super().__init__('sensor_publisher_node')
        
        # الإعلان عن الإعدادات واستلامها
        self.declare_parameter('imu_noise_stddev', 0.05)
        self.declare_parameter('gps_noise_stddev', 0.1)
        self.declare_parameter('temp_noise_stddev', 0.2)
        self.declare_parameter('baro_noise_stddev', 5.0)
        
        self.imu_noise = self.get_parameter('imu_noise_stddev').value
        self.gps_noise = self.get_parameter('gps_noise_stddev').value
        self.temp_noise = self.get_parameter('temp_noise_stddev').value
        self.baro_noise = self.get_parameter('baro_noise_stddev').value

        # إنشاء الناشرين (Publishers)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/sensors/gps', 10)
        self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', 10)
        self.baro_pub = self.create_publisher(FluidPressure, '/sensors/barometer', 10)

        # إنشاء المؤقتات (Timers) بالترددات المطلوبة
        self.create_timer(0.1, self.publish_imu)    # 10 Hz
        self.create_timer(1.0, self.publish_gps)    # 1 Hz
        self.create_timer(0.5, self.publish_temp)   # 2 Hz
        self.create_timer(0.5, self.publish_baro)   # 2 Hz

        self.get_logger().info("Sensor Publisher Node started (Full Version - GPS Fix). Publishing all sensors...")

    def add_noise(self, value, noise_stddev):
        """Helper function to add Gaussian noise."""
        return value + np.random.normal(0.0, noise_stddev)

    def publish_imu(self):
        """Publishes simulated IMU data."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.angular_velocity.x = self.add_noise(0.1, self.imu_noise)
        msg.angular_velocity.y = self.add_noise(-0.1, self.imu_noise)
        msg.angular_velocity.z = self.add_noise(0.05, self.imu_noise)
        msg.linear_acceleration.x = self.add_noise(0.0, self.imu_noise)
        msg.linear_acceleration.y = self.add_noise(0.0, self.imu_noise)
        msg.linear_acceleration.z = self.add_noise(9.81, self.imu_noise)
        self.imu_pub.publish(msg)

    def publish_gps(self):
        """Publishes simulated GPS data."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        
        # --- تعديل ---: إصلاح الخطأ
        # المتغير موجود في 'NavSatStatus' وليس 'NavSatFix'
        msg.status.status = NavSatStatus.STATUS_FIX
        
        msg.latitude = self.add_noise(31.9539, self.gps_noise * 0.00001)
        msg.longitude = self.add_noise(35.9106, self.gps_noise * 0.00001)
        msg.altitude = self.add_noise(750.0, self.gps_noise)
        self.gps_pub.publish(msg)

    def publish_temp(self):
        """Publishes simulated Temperature data."""
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "temp_sensor_link"
        msg.temperature = self.add_noise(25.0, self.temp_noise) # 25 C
        msg.variance = self.temp_noise**2
        self.temp_pub.publish(msg)

    def publish_baro(self):
        """Publishes simulated Barometer/FluidPressure data."""
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "baro_sensor_link"
        msg.fluid_pressure = self.add_noise(92500.0, self.baro_noise) 
        msg.variance = self.baro_noise**2
        self.baro_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None # --- تعديل ---: تعريف المتغير مسبقاً
    try:
        node = SensorPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Sensor Publisher shutting down.") # --- تعديل ---: التحقق قبل التدمير
    except Exception as e:
        if node: node.get_logger().error(f"An error occurred: {e}") # --- تعديل ---: التحقق قبل التسجيل
    finally:
        if node and rclpy.ok(): # --- تعديل ---: التحقق قبل التدمير
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()