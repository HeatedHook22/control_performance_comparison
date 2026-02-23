import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
import matplotlib.pyplot as plt
import time

class local_position_plotter(Node):
    def __init__(self):
        super().__init__('local_position_plotter')
        
        # Subscribe using the Sensor Data QoS profile (Best Effort)
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile_sensor_data
        )
        
        self.times = []
        self.x_data = []
        self.y_data = []
        self.z_data = []
        
        self.start_time = None
        self.get_logger().info('Position Plotter Node started. Recording data...')
        self.get_logger().info('Press Ctrl+C to stop recording and generate graphs.')

    def position_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        self.times.append(current_time)
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)
        self.z_data.append(msg.z)

    def generate_graphs(self):
        if not self.times:
            print("No data was received. Is the topic publishing?")
            return
            
        print('Generating plots...')
        
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle('UAV Local Position vs. Time', fontsize=16)

        ax1.plot(self.times, self.x_data, 'r-', linewidth=2)
        ax1.set_ylabel('X Position (m)')
        ax1.grid(True)

        ax2.plot(self.times, self.y_data, 'g-', linewidth=2)
        ax2.set_ylabel('Y Position (m)')
        ax2.grid(True)

        inverted_z = [-z for z in self.z_data]
        ax3.plot(self.times, inverted_z, 'b-', linewidth=2)
        ax3.set_ylabel('Altitude / -Z (m)')
        ax3.set_xlabel('Time (seconds)')
        ax3.grid(True)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = local_position_plotter()

    try:
        rclpy.spin(plotter_node)
    except KeyboardInterrupt:
        print('\nRecording stopped by user.')
    finally:
        # Generate graphs before destroying the ROS context to avoid logger crashes
        plotter_node.generate_graphs()
        plotter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()