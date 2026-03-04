import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint 
from px4_msgs.msg import VehicleRatesSetpoint 
from px4_msgs.msg import VehicleOdometry # <-- SWAPPED IMPORT
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt
import time
import math
import numpy as np

class data_recorder(Node):
    def __init__(self):
        super().__init__('local_position_plotter')
        
        # Subscribe using the Sensor Data QoS profile (Best Effort)
        self.local_pos_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos_profile_sensor_data
        )

        self.local_pos_sp_subscription = self.create_subscription(
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

        self.rates_sp_subscription = self.create_subscription(
            VehicleRatesSetpoint,
            '/fmu/in/vehicle_rates_setpoint',
            self.rates_setpoint_callback,
            qos_profile_sensor_data
        )

        # Use odometry for angular velocity
        self.angular_velocity_subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.angular_velocity_callback,
            qos_profile_sensor_data
        )

        # Start at 0 (Init)
        self.previous_arming_state = 0
        
        self.position_times = []
        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.position_sp_times = []
        self.x_sp_data = []
        self.y_sp_data = []
        self.z_sp_data = []

        self.attitude_times = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        
        self.attitude_sp_times = []
        self.roll_sp_data = []
        self.pitch_sp_data = []
        self.yaw_sp_data = []

        self.rates_times = []
        self.roll_rate_data = []
        self.pitch_rate_data = []
        self.yaw_rate_data = []

        self.rates_sp_times = []
        self.roll_rate_sp_data = []
        self.pitch_rate_sp_data = []
        self.yaw_rate_sp_data = []

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

    def angular_velocity_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.rates_times.append(current_time)
        self.roll_rate_data.append(math.degrees(msg.angular_velocity[0]))
        self.pitch_rate_data.append(math.degrees(msg.angular_velocity[1]))
        self.yaw_rate_data.append(math.degrees(msg.angular_velocity[2]))

    def rates_setpoint_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.rates_sp_times.append(current_time)
        self.roll_rate_sp_data.append(math.degrees(msg.roll))
        self.pitch_rate_sp_data.append(math.degrees(msg.pitch))
        self.yaw_rate_sp_data.append(math.degrees(msg.yaw))

    def status_callback(self, msg):
        if self.previous_arming_state == 2 and msg.arming_state != 2:
            
            self.get_logger().info("Disarm detected! Stopping recording and generating plots...")
            raise KeyboardInterrupt
        self.previous_arming_state = msg.arming_state

    def calculate_rms(self, actual_times, actual_data, sp_times, sp_data):
        actual_times = np.array(actual_times)
        actual_data = np.array(actual_data)
        sp_times = np.array(sp_times)
        sp_data = np.array(sp_data)
        valid_mask = (actual_times >= sp_times[0]) & (actual_times <= sp_times[-1])
        valid_actual_times = actual_times[valid_mask]
        if len(valid_actual_times) > 1:
            T = valid_actual_times[-1] - valid_actual_times[0]
            if T > 0:
                aligned_sp = np.interp(valid_actual_times, sp_times, sp_data)
                error = actual_data[valid_mask] - aligned_sp
                integral = np.trapz(np.square(error), x=valid_actual_times)
                return np.sqrt(integral / T)
        return None

    def create_plot(self, title, actual_times, actual_data, sp_times, sp_data, rms_texts, ylabels, filename):
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle(title, fontsize=16)

        colors = ['r', 'g', 'b']

        for i, ax in enumerate(axes):
            if sp_times is not None and len(sp_times) > 0 and sp_data is not None:
                ax.plot(sp_times, sp_data[i], f'{colors[i]}--', linewidth=2, label='Setpoint')
            ax.plot(actual_times, actual_data[i], f'{colors[i]}-', linewidth=2, label='Actual')
            ax.set_ylabel(ylabels[i])
            ax.legend(loc="upper right")
            ax.grid(True)
            ax.text(0.02, 0.85, rms_texts[i], transform=ax.transAxes, fontsize=11,
                    bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

        axes[-1].set_xlabel('Time (seconds)')

        fig.tight_layout()
        fig.savefig(filename)

    def generate_metric_graphs(self, metric_name, actual_times, actual_data, sp_times, sp_data, ylabels, rms_unit, file_prefix):
        if not actual_times:
            print(f"No actual data received for {metric_name}. Is the topic publishing?")
            return
            
        print(f'Generating {metric_name} plots...')
        has_setpoints = sp_times is not None and len(sp_times) > 0
        
        rms_texts = ["RMS Error: N/A", "RMS Error: N/A", "RMS Error: N/A"]

        if has_setpoints:
            actual_times_arr = np.array(actual_times)
            sp_times_arr = np.array(sp_times)
            valid_mask = (actual_times_arr >= sp_times_arr[0]) & (actual_times_arr <= sp_times_arr[-1])
            
            for i in range(3):
                rms = self.calculate_rms(actual_times, actual_data[i], sp_times, sp_data[i])
                if rms is not None:
                    rms_texts[i] = f"RMS Error: {rms:.3f}{rms_unit}"
                else:
                    print(f"WARNING: Not enough valid data points to integrate {ylabels[i]}! Skipping RMS.")
        else:
            print(f"WARNING: No {metric_name} setpoints received! Skipping RMS calculation.")

        self.create_plot(
            title=f'UAV {metric_name} vs. Time',
            actual_times=actual_times,
            actual_data=actual_data,
            sp_times=sp_times if has_setpoints else None,
            sp_data=sp_data if has_setpoints else None,
            rms_texts=rms_texts,
            ylabels=ylabels,
            filename=f"data_recording/tmp/previous_run_{file_prefix}.png"
        )

        if has_setpoints:
            cropped_times = actual_times_arr[valid_mask]
            cropped_data = (
                np.array(actual_data[0])[valid_mask],
                np.array(actual_data[1])[valid_mask],
                np.array(actual_data[2])[valid_mask]
            )
            
            self.create_plot(
                title=f'UAV {metric_name} vs. Time (Test Region Only)',
                actual_times=cropped_times,
                actual_data=cropped_data,
                sp_times=sp_times,
                sp_data=sp_data,
                rms_texts=rms_texts,
                ylabels=ylabels,
                filename=f"data_recording/tmp/previous_run_{file_prefix}_cropped.png"
            )

    def generate_graphs(self):
        if not self.position_times or not self.attitude_times:
            print("No data was received. Is the topic publishing?")
            return

        # 1. Generate Attitude Graphs
        self.generate_metric_graphs(
            metric_name="Attitude",
            actual_times=self.attitude_times,
            actual_data=(self.roll_data, self.pitch_data, self.yaw_data),
            sp_times=self.attitude_sp_times,
            sp_data=(self.roll_sp_data, self.pitch_sp_data, self.yaw_sp_data),
            ylabels=('Roll (°)', 'Pitch (°)', 'Yaw (°)'),
            rms_unit="°",
            file_prefix="attitude"
        )

        # 2. Generate Body Rate Graphs
        if self.rates_times:
            self.generate_metric_graphs(
                metric_name="Body Rates",
                actual_times=self.rates_times,
                actual_data=(self.roll_rate_data, self.pitch_rate_data, self.yaw_rate_data),
                sp_times=self.rates_sp_times,
                sp_data=(self.roll_rate_sp_data, self.pitch_rate_sp_data, self.yaw_rate_sp_data),
                ylabels=('Roll Rate (°/s)', 'Pitch Rate (°/s)', 'Yaw Rate (°/s)'),
                rms_unit="°/s",
                file_prefix="rates"
            )

        # 3. Generate Local Position Graphs
        if self.position_times:
            self.generate_metric_graphs(
                metric_name="Local Position",
                actual_times=self.position_times,
                actual_data=(self.x_data, self.y_data, self.z_data),
                sp_times=self.position_setpoint,
                sp_data=(self.roll_rate_sp_data, self.pitch_rate_sp_data, self.yaw_rate_sp_data),
                ylabels=('Roll Rate (°/s)', 'Pitch Rate (°/s)', 'Yaw Rate (°/s)'),
                rms_unit="°/s",
                file_prefix="rates"
            )

        # plt.show()

def main(args=None):
    rclpy.init(args=args)
    data_recorder_node = data_recorder()

    try:
        rclpy.spin(data_recorder_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nRecording stopped by user.')
    finally:
        # Generate graphs before destroying the ROS context to avoid logger crashes
        data_recorder_node.generate_graphs()
        data_recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()