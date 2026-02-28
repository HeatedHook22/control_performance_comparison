import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint # <-- ADDED IMPORT
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt
import time
import math
import numpy as np

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

        self.attitude_sp_subscription = self.create_subscription(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint_v1',
            self.attitude_setpoint_callback,
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
        
        self.attitude_sp_times = []
        self.roll_sp_data = []
        self.pitch_sp_data = []
        self.yaw_sp_data = []

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

    # SETPOINT CALLBACK (Identical math, but uses msg.q_d)
    def attitude_setpoint_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        q = msg.q_d # Notice this is q_d for desired quaternion
        
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) 
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.attitude_sp_times.append(current_time)
        self.roll_sp_data.append(math.degrees(roll))
        self.pitch_sp_data.append(math.degrees(pitch))
        self.yaw_sp_data.append(math.degrees(yaw))

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

        # Safety check: Did we actually receive any attitude setpoints before shutting down?
        has_setpoints = len(self.attitude_sp_times) > 0

        # --- CALCULATE RMS ERROR ---
        if has_setpoints:
            actual_times = np.array(self.attitude_times)
            sp_times = np.array(self.attitude_sp_times)
            
            # Create a mask to only evaluate actual data while setpoints were being published
            valid_mask = (actual_times >= sp_times[0]) & (actual_times <= sp_times[-1])
            valid_actual_times = actual_times[valid_mask]
            
            # Ensure we have enough points to actually calculate an integral
            if len(valid_actual_times) > 1:
                # Calculate total duration T
                T = valid_actual_times[-1] - valid_actual_times[0]
                
                # Roll
                aligned_sp_roll = np.interp(valid_actual_times, sp_times, np.array(self.roll_sp_data))
                roll_error = np.array(self.roll_data)[valid_mask] - aligned_sp_roll
                roll_integral = np.trapz(np.square(roll_error), x=valid_actual_times)
                rms_roll = np.sqrt(roll_integral / T)
                rms_text_roll = f"RMS Error: {rms_roll:.3f}°"

                # Pitch
                aligned_sp_pitch = np.interp(valid_actual_times, sp_times, np.array(self.pitch_sp_data))
                pitch_error = np.array(self.pitch_data)[valid_mask] - aligned_sp_pitch
                pitch_integral = np.trapz(np.square(pitch_error), x=valid_actual_times)
                rms_pitch = np.sqrt(pitch_integral / T)
                rms_text_pitch = f"RMS Error: {rms_pitch:.3f}°"

                # Yaw
                aligned_sp_yaw = np.interp(valid_actual_times, sp_times, np.array(self.yaw_sp_data))
                yaw_error = np.array(self.yaw_data)[valid_mask] - aligned_sp_yaw 
                yaw_integral = np.trapz(np.square(yaw_error), x=valid_actual_times)
                rms_yaw = np.sqrt(yaw_integral / T)
                rms_text_yaw = f"RMS Error: {rms_yaw:.3f}°"
            else:
                print("WARNING: Not enough valid data points to integrate! Skipping RMS.")
                rms_text_roll = "RMS Error: N/A"
                rms_text_pitch = "RMS Error: N/A"
                rms_text_yaw = "RMS Error: N/A"
        else:
            print("WARNING: No attitude setpoints received! Skipping RMS calculation.")
            rms_text_roll = "RMS Error: N/A"
            rms_text_pitch = "RMS Error: N/A"
            rms_text_yaw = "RMS Error: N/A"

        # --- ATTITUDE GRAPH (FULL DATA) ---
        fig2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig2.suptitle('UAV Attitude vs. Time', fontsize=16)

        # Plot Roll
        if has_setpoints:
            ax4.plot(self.attitude_sp_times, self.roll_sp_data, 'r--', linewidth=2, label='Setpoint')
        ax4.plot(self.attitude_times, self.roll_data, 'r-', linewidth=2, label='Actual')
        ax4.set_ylabel('Roll (deg)')
        ax4.legend(loc="upper right")
        ax4.grid(True)
        ax4.text(0.02, 0.85, rms_text_roll, transform=ax4.transAxes, fontsize=11,
                 bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

        # Plot Pitch
        if has_setpoints:
            ax5.plot(self.attitude_sp_times, self.pitch_sp_data, 'g--', linewidth=2, label='Setpoint')
        ax5.plot(self.attitude_times, self.pitch_data, 'g-', linewidth=2, label='Actual')
        ax5.set_ylabel('Pitch (deg)')
        ax5.legend(loc="upper right")
        ax5.grid(True)
        ax5.text(0.02, 0.85, rms_text_pitch, transform=ax5.transAxes, fontsize=11,
                 bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

        # Plot Yaw
        if has_setpoints:
            ax6.plot(self.attitude_sp_times, self.yaw_sp_data, 'b--', linewidth=2, label='Setpoint')
        ax6.plot(self.attitude_times, self.yaw_data, 'b-', linewidth=2, label='Actual')
        ax6.set_ylabel('Yaw (deg)')
        ax6.set_xlabel('Time (seconds)')
        ax6.legend(loc="upper right")
        ax6.grid(True)
        ax6.text(0.02, 0.85, rms_text_yaw, transform=ax6.transAxes, fontsize=11,
                 bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

        fig2.tight_layout()
        fig2.savefig("data_recording/tmp/previous_run_attitude.png")

        # --- CROPPED ATTITUDE GRAPH (SETPOINT REGION ONLY) ---
        if has_setpoints:
            fig3, (ax7, ax8, ax9) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            fig3.suptitle('UAV Attitude vs. Time (Test Region Only)', fontsize=16)

            cropped_times = np.array(self.attitude_times)[valid_mask]

            # Plot Roll
            ax7.plot(self.attitude_sp_times, self.roll_sp_data, 'r--', linewidth=2, label='Setpoint')
            ax7.plot(cropped_times, np.array(self.roll_data)[valid_mask], 'r-', linewidth=2, label='Actual')
            ax7.set_ylabel('Roll (deg)')
            ax7.legend(loc="upper right")
            ax7.grid(True)
            ax7.text(0.02, 0.85, rms_text_roll, transform=ax7.transAxes, fontsize=11,
                     bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

            # Plot Pitch
            ax8.plot(self.attitude_sp_times, self.pitch_sp_data, 'g--', linewidth=2, label='Setpoint')
            ax8.plot(cropped_times, np.array(self.pitch_data)[valid_mask], 'g-', linewidth=2, label='Actual')
            ax8.set_ylabel('Pitch (deg)')
            ax8.legend(loc="upper right")
            ax8.grid(True)
            ax8.text(0.02, 0.85, rms_text_pitch, transform=ax8.transAxes, fontsize=11,
                     bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

            # Plot Yaw
            ax9.plot(self.attitude_sp_times, self.yaw_sp_data, 'b--', linewidth=2, label='Setpoint')
            ax9.plot(cropped_times, np.array(self.yaw_data)[valid_mask], 'b-', linewidth=2, label='Actual')
            ax9.set_ylabel('Yaw (deg)')
            ax9.set_xlabel('Time (seconds)')
            ax9.legend(loc="upper right")
            ax9.grid(True)
            ax9.text(0.02, 0.85, rms_text_yaw, transform=ax9.transAxes, fontsize=11,
                     bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

            fig3.tight_layout()
            fig3.savefig("data_recording/tmp/previous_run_attitude_cropped.png")

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