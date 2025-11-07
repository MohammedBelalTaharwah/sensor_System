import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature, FluidPressure
from collections import deque
# --- تم الحذف ---: تم حذف 'from rclpy.qos import SensorDataQoS'

class VisualizationNode(Node):
    """
    Plots BOTH raw and filtered sensor data for comparison.
    """
    
    SEA_LEVEL_PRESSURE_PA = 101325.0  # (Pa)
    
    def __init__(self):
        super().__init__('visualization_node')
        
        self.max_plot_points = 100 
        
        self.time_data = deque(maxlen=self.max_plot_points)
        self.fused_altitude_data = deque(maxlen=self.max_plot_points)
        self.raw_altitude_data = deque(maxlen=self.max_plot_points)
        self.fused_temp_data = deque(maxlen=self.max_plot_points)
        self.raw_temp_data = deque(maxlen=self.max_plot_points)
        
        self.start_time = self.get_clock().now().nanoseconds

        self.current_fused_altitude = np.nan
        self.current_raw_altitude = np.nan
        self.current_fused_temp = np.nan
        self.current_raw_temp = np.nan

        # --- تم الحذف ---: تم حذف 'qos_profile = SensorDataQoS()'

        # 1. المفلتر (من 'sensor_fusion')
        self.create_subscription(
            Float64, '/fused/altitude', self.fused_altitude_callback, 10)
        self.create_subscription(
            Float64, '/fused/temperature', self.fused_temp_callback, 10)
            
        # 2. الخام (من 'sensor_publisher')
        # --- تم التعديل ---: العودة لاستخدام '10'
        self.create_subscription(
            Temperature, '/sensors/temperature', self.raw_temp_callback, 10)
        self.create_subscription(
            FluidPressure, '/sensors/barometer', self.raw_pressure_callback, 10)

        # ... (باقي الكود الخاص بالرسم البياني يبقى كما هو تماماً) ...
        
        plt.ion() 
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.fig.suptitle('Sensor Fusion Visualization (Raw vs. Filtered)')
        
        self.line_raw_alt, = self.ax1.plot([], [], 'r--', label='Raw Altitude (Noisy)') 
        self.line_fused_alt, = self.ax1.plot([], [], 'b-', label='Filtered Altitude (Clean)') 
        self.ax1.set_ylabel('Altitude (m)')
        self.ax1.legend(loc='upper left') 
        self.ax1.grid()
        
        self.line_raw_temp, = self.ax2.plot([], [], 'r--', label='Raw Temp (Noisy)') 
        self.line_fused_temp, = self.ax2.plot([], [], 'b-', label='Filtered Temp (Clean)') 
        self.ax2.set_ylabel('Temperature (C)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.legend(loc='upper left') 
        self.ax2.grid()
        
        self.plot_update_timer = self.create_timer(0.2, self.update_plot) # 5 Hz
        self.get_logger().info("Visualization Node started. Plotting Raw vs. Filtered data...")

    def get_current_time_sec(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    def fused_altitude_callback(self, msg: Float64):
        self.current_fused_altitude = msg.data

    def fused_temp_callback(self, msg: Float64):
        self.current_fused_temp = msg.data

    def raw_temp_callback(self, msg: Temperature):
        self.current_raw_temp = msg.temperature

    def raw_pressure_callback(self, msg: FluidPressure):
        try:
            pressure_pa = msg.fluid_pressure
            pressure_ratio = pressure_pa / self.SEA_LEVEL_PRESSURE_PA
            raw_altitude = 44330.0 * (1.0 - (pressure_ratio ** 0.190284))
            self.current_raw_altitude = raw_altitude
        except Exception:
            self.current_raw_altitude = np.nan

    def update_plot(self):
        try:
            self.time_data.append(self.get_current_time_sec())
            
            self.fused_altitude_data.append(self.current_fused_altitude)
            self.fused_temp_data.append(self.current_fused_temp)
            self.raw_altitude_data.append(self.current_raw_altitude)
            self.raw_temp_data.append(self.current_raw_temp)

            self.line_raw_alt.set_data(self.time_data, self.raw_altitude_data)
            self.line_fused_alt.set_data(self.time_data, self.fused_altitude_data)
            self.ax1.relim()
            self.ax1.autoscale_view()

            self.line_raw_temp.set_data(self.time_data, self.raw_temp_data)
            self.line_fused_temp.set_data(self.time_data, self.fused_temp_data)
            self.ax2.relim()
            self.ax2.autoscale_view()

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001) 
            
        except Exception as e:
            self.get_logger().error(f"Error in update_plot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Visualization shutting down (Ctrl+C).")
    except Exception as e:
        if node:
            node.get_logger().error(f"An error occurred: {e}")
    finally:
        plt.ioff()
        if node and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()