import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt
import time
import math

class local_position_plotter(Node):
    def __init__(self):
        super().__init__('local_position_plotter')
        
        # Subscribe using the Sensor Data QoS profile (Best Effort)
        self.local_pos_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile_sensor_data
        )

        self.offboard_mode_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_profile_sensor_data
        )

        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile_sensor_data
        )

        # Start at 0 (Init)
        self.previous_arming_state = 0
        
        self.position_times = []
        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.attitude_times = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        
        self.start_time = None
        self.get_logger().info('Position Plotter Node started. Recording data...')
        self.get_logger().info('Press Ctrl+C to stop recording and generate graphs.')

    def position_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        self.position_times.append(current_time)
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)
        self.z_data.append(msg.z)

    def attitude_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        # Convert PX4 Quaternion (w, x, y, z) to Euler Angles (Roll, Pitch, Yaw)
        q = msg.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # Use 90 degrees if out of bounds
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.attitude_times.append(current_time)
        self.roll_data.append(math.degrees(roll))
        self.pitch_data.append(math.degrees(pitch))
        self.yaw_data.append(math.degrees(yaw))

    def status_callback(self, msg):
        if self.previous_arming_state == 2 and msg.arming_state != 2:
            
            self.get_logger().info("Disarm detected! Stopping recording and generating plots...")
            raise KeyboardInterrupt
        self.previous_arming_state = msg.arming_state

    def generate_graphs(self):
        if not self.position_times or not self.attitude_times:
            print("No data was received. Is the topic publishing?")
            return
            
        print('Generating plots...')
        
        # --- POSITION GRAPH ---
        # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        # fig.suptitle('UAV Local Position vs. Time', fontsize=16)

        # ax1.plot(self.position_times, self.x_data, 'r-', linewidth=2)
        # ax1.set_ylabel('X Position (m)')
        # ax1.grid(True)

        # ax2.plot(self.position_times, self.y_data, 'g-', linewidth=2)
        # ax2.set_ylabel('Y Position (m)')
        # ax2.grid(True)

        # inverted_z = [-z for z in self.z_data]
        # ax3.plot(self.position_times, inverted_z, 'b-', linewidth=2)
        # ax3.set_ylabel('Altitude / -Z (m)')
        # ax3.set_xlabel('Time (seconds)')
        # ax3.grid(True)

        # plt.tight_layout()
        # plt.savefig("data_recording/tmp/previous_run_position.png")

        # --- ATTITUDE GRAPH ---
        fig2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig2.suptitle('UAV Attitude vs. Time', fontsize=16)

        ax4.plot(self.attitude_times, self.roll_data, 'r-', linewidth=2)
        ax4.set_ylabel('Roll (deg)')
        ax4.grid(True)

        ax5.plot(self.attitude_times, self.pitch_data, 'g-', linewidth=2)
        ax5.set_ylabel('Pitch (deg)')
        ax5.grid(True)

        ax6.plot(self.attitude_times, self.yaw_data, 'b-', linewidth=2)
        ax6.set_ylabel('Yaw (deg)')
        ax6.set_xlabel('Time (seconds)')
        ax6.grid(True)

        fig2.tight_layout()
        fig2.savefig("data_recording/tmp/previous_run_attitude.png")

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = local_position_plotter()

    try:
        rclpy.spin(plotter_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nRecording stopped by user.')
    finally:
        # Generate graphs before destroying the ROS context to avoid logger crashes
        plotter_node.generate_graphs()
        plotter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()