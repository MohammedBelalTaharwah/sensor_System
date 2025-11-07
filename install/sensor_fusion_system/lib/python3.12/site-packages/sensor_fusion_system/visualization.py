import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64
from collections import deque

class VisualizationNode(Node):
    """
    Subscribes to fused sensor data (/fused/altitude and /fused/temperature)
    and plots it over time in real-time using Matplotlib.
    """
    def __init__(self):
        super().__init__('visualization_node')
        
        self.max_plot_points = 100 # How many points to show on the plot
        
        # Data storage deques (thread-safe and efficient for fixed-size lists)
        self.time_data = deque(maxlen=self.max_plot_points)
        self.altitude_data = deque(maxlen=self.max_plot_points)
        self.temp_data = deque(maxlen=self.max_plot_points)
        
        # Store the start time to create a relative time axis (in seconds)
        self.start_time = self.get_clock().now().nanoseconds

        # Store the last received value for downsampling
        self.current_altitude = np.nan
        self.current_temp = np.nan

        # Subscribers (use default QoS or SensorDataQoS)
        self.create_subscription(
            Float64, '/fused/altitude', self.altitude_callback, 10)
        self.create_subscription(
            Float64, '/fused/temperature', self.temp_callback, 10)

        # --- Matplotlib Setup ---
        # plt.ion() is CRITICAL for non-blocking plots in a ROS node
        plt.ion() 
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.fig.suptitle('Sensor Fusion Visualization (Real-Time)')
        
        # Setup Altitude Plot (ax1)
        self.line_alt, = self.ax1.plot([], [], 'r-', label='Altitude') # 'r-' = red line
        self.ax1.set_ylabel('Altitude (m)')
        self.ax1.legend(loc='upper left')
        self.ax1.grid()
        
        # Setup Temperature Plot (ax2)
        self.line_temp, = self.ax2.plot([], [], 'b-', label='Temperature') # 'b-' = blue line
        self.ax2.set_ylabel('Temperature (C)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.legend(loc='upper left')
        self.ax2.grid()
        
        # --- End Matplotlib Setup ---

        # A ROS Timer to update the plot at a fixed rate (e.g., 5 Hz)
        # We do this in a timer, NOT in the callbacks, to avoid blocking
        self.plot_update_timer = self.create_timer(0.2, self.update_plot) # 5 Hz

        self.get_logger().info("Visualization Node started. Plotting data...")

    def get_current_time_sec(self):
        """Returns time in seconds since the node started."""
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    def altitude_callback(self, msg: Float64):
        """Callback to store the latest altitude data."""
        self.current_altitude = msg.data

    def temp_callback(self, msg: Float64):
        """Callback to store the latest temperature data."""
        self.current_temp = msg.data

    def update_plot(self):
        """
        Timer callback to update the matplotlib plot.
        This function "downsamples" the data to the timer's frequency.
        """
        try:
            # Append the latest data (or np.nan if no new data arrived)
            self.time_data.append(self.get_current_time_sec())
            self.altitude_data.append(self.current_altitude)
            self.temp_data.append(self.current_temp)

            # Reset current values to NaN to avoid plotting stale data
            # This makes the plot show gaps if a sensor stops sending data
            self.current_altitude = np.nan
            self.current_temp = np.nan

            # Update altitude plot data
            self.line_alt.set_data(self.time_data, self.altitude_data)
            self.ax1.relim()      # Recalculate limits
            self.ax1.autoscale_view() # Autoscale

            # Update temperature plot data
            self.line_temp.set_data(self.time_data, self.temp_data)
            self.ax2.relim()
            self.ax2.autoscale_view()

            # Redraw the canvas
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
            # A tiny pause is needed for the GUI to update
            plt.pause(0.001) 
            
        except Exception as e:
            self.get_logger().error(f"Error in update_plot: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualizationNode()
        # rclpy.spin() will block. Matplotlib updates happen in the timer.
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Visualization shutting down (Ctrl+C).")
    except Exception as e:
        if node:
            node.get_logger().error(f"An error occurred: {e}")
    finally:
        # Graceful shutdown
        plt.ioff() # Turn off interactive mode
        # You can optionally add plt.show() here to keep the final plot window open
        # plt.show() 
        if node and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()