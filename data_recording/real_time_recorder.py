import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint 
from px4_msgs.msg import VehicleRatesSetpoint 
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt
import time
import math
import numpy as np
import csv
import itertools
import os

class Series3D:
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.z = []

    def append(self, t, x, y, z):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def data(self):
        return (self.x, self.y, self.z)

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
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.position_setpoint_callback,
            qos_profile_sensor_data
        )

        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
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

        self.offboard_mode_subscription = self.create_subscription(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self.offboard_mode_callback,
            qos_profile_sensor_data
        )

        self.thrust_subscription = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/out/vehicle_thrust_setpoint',
            self.thrust_callback,
            qos_profile_sensor_data
        )

        self.thrust_sp_subscription = self.create_subscription(
            VehicleThrustSetpoint,
            '/fmu/in/vehicle_thrust_setpoint',
            self.thrust_sp_callback,
            qos_profile_sensor_data
        )

        self.torque_subscription = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/out/vehicle_torque_setpoint',
            self.torque_callback,
            qos_profile_sensor_data
        )

        self.torque_sp_subscription = self.create_subscription(
            VehicleTorqueSetpoint,
            '/fmu/in/vehicle_torque_setpoint',
            self.torque_sp_callback,
            qos_profile_sensor_data
        )

        # Track exact test region from control flags
        self.test_mode_detected = None
        self.test_region_times = []

        # Start at 0 (Init)
        self.previous_arming_state = 0
        
        # Data storage containers using helper class
        self.pos = Series3D()
        self.pos_sp = Series3D()
        
        self.att = Series3D()
        self.att_sp = Series3D()
        
        self.rates = Series3D()
        self.rates_sp = Series3D()
        
        self.thrust = Series3D()
        self.thrust_sp = Series3D()
        
        self.torque = Series3D()
        self.torque_sp = Series3D()

        self.start_time = None
        self.get_logger().info('Position Plotter Node started. Recording data...')
        self.get_logger().info('Press Ctrl+C to stop recording and generate graphs.')

    def position_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.pos.append(current_time, msg.x, msg.y, msg.z)

    def position_setpoint_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()

        current_time = time.time() - self.start_time
        self.pos_sp.append(current_time, msg.position[0], msg.position[1], msg.position[2])

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

        self.att.append(current_time, math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

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

        self.att_sp.append(current_time, math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def angular_velocity_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.rates.append(current_time, math.degrees(msg.angular_velocity[0]), math.degrees(msg.angular_velocity[1]), math.degrees(msg.angular_velocity[2]))

    def rates_setpoint_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.rates_sp.append(current_time, math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw))

    def thrust_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.thrust.append(current_time, msg.xyz[0], msg.xyz[1], msg.xyz[2])

    def thrust_sp_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.thrust_sp.append(current_time, msg.xyz[0], msg.xyz[1], msg.xyz[2])

    def torque_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.torque.append(current_time, msg.xyz[0], msg.xyz[1], msg.xyz[2])

    def torque_sp_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        self.torque_sp.append(current_time, msg.xyz[0], msg.xyz[1], msg.xyz[2])

    def status_callback(self, msg):
        if self.previous_arming_state == 2 and msg.arming_state != 2:
            
            self.get_logger().info("Disarm detected! Stopping recording and generating plots...")
            raise KeyboardInterrupt
        self.previous_arming_state = msg.arming_state

    def offboard_mode_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        current_time = time.time() - self.start_time
        
        # Assuming Position = True is used for staging, the pure test starts when position drops
        if msg.body_rate and not msg.attitude and not msg.position:
            self.test_mode_detected = "body_rate"
            self.test_region_times.append(current_time)
        elif msg.attitude and not msg.position:
            self.test_mode_detected = "attitude"
            self.test_region_times.append(current_time)

    def get_test_window(self):
        if self.test_mode_detected and self.test_region_times:
            self.get_logger().info(f"Test region formally detected via OffboardControlMode: {self.test_mode_detected}")
            return self.test_region_times[0], self.test_region_times[-1], self.test_mode_detected

        # Fallbacks just in case the offboard mode topic isn't found
        if self.rates_sp.t:
            self.get_logger().info("Fallback: Test region anchored to Body Rates setpoints.")
            return self.rates_sp.t[0], self.rates_sp.t[-1], "body_rate"
        elif self.att_sp.t:
            self.get_logger().info("Fallback: Test region anchored to Attitude setpoints.")
            return self.att_sp.t[0], self.att_sp.t[-1], "attitude"
        elif self.pos_sp.t:
            self.get_logger().info("Fallback: Test region anchored to Position setpoints.")
            return self.pos_sp.t[0], self.pos_sp.t[-1], "position"
        return None, None, None

    def calculate_rms(self, actual_times, actual_data, sp_times, sp_data, test_start, test_end, fraction_of_rise_time = 0.95):
        actual_times = np.array(actual_times)
        actual_data = np.array(actual_data)
        sp_times = np.array(sp_times)
        sp_data = np.array(sp_data)
        
        # Restrict analysis strictly to the global test window
        start_bound = max(sp_times[0], test_start) if test_start is not None else sp_times[0]
        end_bound = min(sp_times[-1], test_end) if test_end is not None else sp_times[-1]
        
        valid_mask = (actual_times >= start_bound) & (actual_times <= end_bound)
        valid_actual_times = actual_times[valid_mask]
        
        if len(valid_actual_times) > 1:
            aligned_sp = np.interp(valid_actual_times, sp_times, sp_data)
            error = actual_data[valid_mask] - aligned_sp
            
            # Find the 95% steady-state starting point WITHIN the test window
            final_sp = aligned_sp[-1]
            initial_actual = actual_data[valid_mask][0]
            step_range = final_sp - initial_actual
            
            steady_idx = 0
            if abs(step_range) > 1e-3: # Ignore if it wasn't a real step command
                threshold = initial_actual + fraction_of_rise_time * step_range
                if step_range > 0:
                    crossings = np.where(actual_data[valid_mask] >= threshold)[0]
                else:
                    crossings = np.where(actual_data[valid_mask] <= threshold)[0]
                
                if len(crossings) > 0:
                    steady_idx = crossings[0]
            
            ss_times = valid_actual_times[steady_idx:]
            ss_error = error[steady_idx:]
            
            T = ss_times[-1] - ss_times[0]
            if T > 0 and len(ss_times) > 1:
                integral = np.trapz(np.square(ss_error), x=ss_times)
                rms = np.sqrt(integral / T)
                return rms, valid_actual_times[steady_idx]
                
        return None, None

    def create_plot(self, title, plot_runs, ylabels, filename):
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(title, fontsize=16)

        for i, ax in enumerate(axes):
            rms_box_texts = []
            
            for run in plot_runs:
                a_t = run.get('actual_times', [])
                if len(a_t) == 0:
                    continue
                    
                a_d = run['actual_data'][i]
                sp_t = run.get('sp_times', [])
                sp_d = run.get('sp_data', [])
                
                c = run.get('color', ['r', 'g', 'b'][i])
                prefix = run.get('label_prefix', '')
                
                sp_label = f'{prefix} Setpoint'.strip()
                act_label = f'{prefix} Actual'.strip()
                rms_label = f'{prefix} RMS Start'.strip()

                if sp_t is not None and len(sp_t) > 0 and sp_d is not None:
                    ax.plot(sp_t, sp_d[i], color=c, linestyle='--', linewidth=1.5, alpha=0.7, label=sp_label)
                
                ax.plot(a_t, a_d, color=c, linestyle='-', linewidth=2, label=act_label)
                
                rms_starts = run.get('rms_start_times')
                if rms_starts is not None and rms_starts[i] is not None:
                    line_color = 'black' if not prefix else c
                    ax.axvline(x=rms_starts[i], color=line_color, linestyle=':', alpha=0.7, label=rms_label)
                
                rms_texts = run.get('rms_texts')
                if rms_texts is not None:
                    rms_box_texts.append(f"{prefix + ' ' if prefix else ''}{rms_texts[i]}")
            
            ax.set_ylabel(ylabels[i])
            
            handles, labels_lgd = ax.get_legend_handles_labels()
            by_label = dict(zip(labels_lgd, handles))
            ax.legend(by_label.values(), by_label.keys(), loc="upper left", bbox_to_anchor=(1.02, 1), 
                      fontsize=8 if len(plot_runs) > 1 else 10)
            ax.grid(True)
            
            if rms_box_texts:
                ax.text(1.02, 0.05, "\n".join(rms_box_texts), transform=ax.transAxes, fontsize=10,
                        verticalalignment='bottom', bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))

        axes[-1].set_xlabel('Time (seconds)')
        fig.tight_layout(rect=[0, 0, 0.82, 1])
        fig.savefig(filename, dpi=300)

    def generate_metric_graphs(self, metric_name, actual_times, actual_data, sp_times, sp_data, ylabels, rms_unit, file_prefix, global_start, global_end):
        if not actual_times:
            print(f"No actual data received for {metric_name}. Is the topic publishing?")
            return
            
        print(f'Generating {metric_name} plots...')
        has_setpoints = sp_times is not None and len(sp_times) > 0
        
        rms_texts = ["RMS Error: N/A", "RMS Error: N/A", "RMS Error: N/A"]
        rms_start_times = [None, None, None]

        if has_setpoints:
            actual_times_arr = np.array(actual_times)
            sp_times_arr = np.array(sp_times)
            valid_mask = (actual_times_arr >= sp_times_arr[0]) & (actual_times_arr <= sp_times_arr[-1])
            
            for i in range(3):
                rms, ss_time = self.calculate_rms(actual_times, actual_data[i], sp_times, sp_data[i], global_start, global_end)
                if rms is not None:
                    rms_texts[i] = f"RMS Error: {rms:.3f}{rms_unit}"
                    rms_start_times[i] = ss_time
                else:
                    print(f"WARNING: Not enough valid data points to integrate {ylabels[i]}! Skipping RMS.")
        else:
            print(f"WARNING: No {metric_name} setpoints received! Skipping RMS calculation.")

        run_full = {
            'actual_times': actual_times,
            'actual_data': actual_data,
            'sp_times': sp_times if has_setpoints else None,
            'sp_data': sp_data if has_setpoints else None,
            'rms_texts': rms_texts,
            'rms_start_times': rms_start_times
        }

        self.create_plot(
            title=f'UAV {metric_name} vs. Time',
            plot_runs=[run_full],
            ylabels=ylabels,
            filename=f"data_recording/tmp/previous_run_{file_prefix}.png"
        )

        if has_setpoints and global_start is not None and global_end is not None:
            actual_times_arr = np.array(actual_times)
            sp_times_arr = np.array(sp_times)
            
            global_mask = (actual_times_arr >= global_start) & (actual_times_arr <= global_end)
            sp_global_mask = (sp_times_arr >= global_start) & (sp_times_arr <= global_end)
            
            cropped_times = actual_times_arr[global_mask] - global_start
            cropped_data = (
                np.array(actual_data[0])[global_mask],
                np.array(actual_data[1])[global_mask],
                np.array(actual_data[2])[global_mask]
            )
            
            cropped_sp_times = sp_times_arr[sp_global_mask] - global_start
            cropped_sp_data = (
                np.array(sp_data[0])[sp_global_mask],
                np.array(sp_data[1])[sp_global_mask],
                np.array(sp_data[2])[sp_global_mask]
            )
            
            cropped_rms_starts = [t - global_start if t is not None else None for t in rms_start_times]
            
            run_cropped = {
                'actual_times': cropped_times,
                'actual_data': cropped_data,
                'sp_times': cropped_sp_times,
                'sp_data': cropped_sp_data,
                'rms_texts': rms_texts,
                'rms_start_times': cropped_rms_starts
            }

            self.create_plot(
                title=f'UAV {metric_name} vs. Time (Test Region Only)',
                plot_runs=[run_cropped],
                ylabels=ylabels,
                filename=f"data_recording/tmp/previous_run_{file_prefix}_cropped.png"
            )

    def export_test_region_to_csv(self, global_start, global_end, filename, metrics_list):
        if global_start is None or global_end is None:
            return

        columns = {}

        for m in metrics_list:
            times = m['actual_times']
            data_tuple = m['actual_data']
            sp_times = m['sp_times']
            sp_data_tuple = m['sp_data']
            prefix = m['csv_prefix']
            labels = m['labels']
            
            if not times: continue
            
            t_arr = np.array(times)
            mask = (t_arr >= global_start) & (t_arr <= global_end)
            columns[f'{prefix}_Time'] = t_arr[mask] - global_start
            
            for i, d in enumerate(data_tuple):
                columns[f'{prefix}_{labels[i]}'] = np.array(d)[mask]
            
            if sp_times and len(sp_times) > 0:
                spt_arr = np.array(sp_times)
                sp_mask = (spt_arr >= global_start) & (spt_arr <= global_end)
                columns[f'{prefix}_SP_Time'] = spt_arr[sp_mask] - global_start
                for i, d in enumerate(sp_data_tuple):
                    columns[f'{prefix}_SP_{labels[i]}'] = np.array(d)[sp_mask]

        if not columns:
            return

        keys = list(columns.keys())
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(keys)
            writer.writerows(itertools.zip_longest(*[columns[k] for k in keys], fillvalue=''))

    def generate_comparison_graphs(self, metrics_list):
        att_file = "data_recording/tmp/attitude_test_region.csv"
        br_file = "data_recording/tmp/body_rate_test_region.csv"
        
        if not (os.path.exists(att_file) and os.path.exists(br_file)):
            return
            
        self.get_logger().info("Both Attitude and Body Rate CSVs found. Generating comparison plots...")
        
        def load_csv(filepath):
            data = {}
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                try:
                    headers = next(reader)
                    for h in headers:
                        data[h] = []
                    for row in reader:
                        for i, val in enumerate(row):
                            if val.strip() != '':
                                data[headers[i]].append(float(val))
                except StopIteration:
                    pass
            for k in data:
                data[k] = np.array(data[k])
            return data
            
        att_data = load_csv(att_file)
        br_data = load_csv(br_file)
        
        def extract_run(data_dict, prefix, labels, rms_unit, color, label_prefix):
            t = data_dict.get(f'{prefix}_Time', np.array([]))
            if len(t) == 0:
                return None
            d = [data_dict.get(f'{prefix}_{l}', np.array([])) for l in labels]
            spt = data_dict.get(f'{prefix}_SP_Time', np.array([]))
            spd = [data_dict.get(f'{prefix}_SP_{l}', np.array([])) for l in labels]
            
            rms_texts = ["N/A", "N/A", "N/A"]
            rms_starts = [None, None, None]
            if len(spt) > 0:
                for i in range(3):
                    rms, ss_time = self.calculate_rms(t, d[i], spt, spd[i], 0, None)
                    if rms is not None:
                        rms_texts[i] = f"RMS Error: {rms:.3f}{rms_unit}"
                        rms_starts[i] = ss_time
            
            return {
                'actual_times': t,
                'actual_data': d,
                'sp_times': spt,
                'sp_data': spd,
                'color': color,
                'label_prefix': label_prefix,
                'rms_texts': rms_texts,
                'rms_start_times': rms_starts
            }

        for m in metrics_list:
            prefix = m['csv_prefix']
            
            if f'{prefix}_Time' not in att_data and f'{prefix}_Time' not in br_data:
                continue
            
            runs = []
            att_run = extract_run(att_data, prefix, m['labels'], m['rms_unit'], 'blue', 'Att Mode')
            if att_run: runs.append(att_run)
            
            br_run = extract_run(br_data, prefix, m['labels'], m['rms_unit'], 'orange', 'BR Mode')
            if br_run: runs.append(br_run)
            
            self.create_plot(
                title=f"UAV {m['metric_name']} vs. Time (Attitude vs Body Rate Mode)",
                plot_runs=runs,
                ylabels=m['ylabels'],
                filename=f"data_recording/tmp/comparison_{m['file_prefix']}.png"
            )

    def generate_graphs(self):
        if not self.pos.t or not self.att.t:
            print("No data was received. Is the topic publishing?")
            return

        global_start, global_end, test_mode = self.get_test_window()

        # CENTRALIZED METRIC CONFIGURATION
        metrics_list = []

        metrics_list.append({
            'metric_name': "Attitude",
            'file_prefix': "attitude",
            'csv_prefix': "Attitude",
            'labels': ["Roll", "Pitch", "Yaw"],
            'ylabels': ('Roll (°)', 'Pitch (°)', 'Yaw (°)'),
            'rms_unit': "°",
            'actual_times': self.att.t,
            'actual_data': self.att.data(),
            'sp_times': self.att_sp.t,
            'sp_data': self.att_sp.data()
        })

        if self.rates.t:
            metrics_list.append({
                'metric_name': "Body Rates",
                'file_prefix': "rates",
                'csv_prefix': "Rates",
                'labels': ["RollRate", "PitchRate", "YawRate"],
                'ylabels': ('Roll Rate (°/s)', 'Pitch Rate (°/s)', 'Yaw Rate (°/s)'),
                'rms_unit': "°/s",
                'actual_times': self.rates.t,
                'actual_data': self.rates.data(),
                'sp_times': self.rates_sp.t,
                'sp_data': self.rates_sp.data()
            })

        if self.pos.t:
            metrics_list.append({
                'metric_name': "Local Position",
                'file_prefix': "position",
                'csv_prefix': "Position",
                'labels': ["X", "Y", "Z"],
                'ylabels': ('x (m)', 'y (m)', 'z (m)'),
                'rms_unit': "m",
                'actual_times': self.pos.t,
                'actual_data': self.pos.data(),
                'sp_times': self.pos_sp.t,
                'sp_data': self.pos_sp.data()
            })

        if self.thrust.t:
            metrics_list.append({
                'metric_name': "Thrust",
                'file_prefix': "thrust",
                'csv_prefix': "Thrust",
                'labels': ["X", "Y", "Z"],
                'ylabels': ('Thrust X (norm)', 'Thrust Y (norm)', 'Thrust Z (norm)'),
                'rms_unit': "",
                'actual_times': self.thrust.t,
                'actual_data': self.thrust.data(),
                'sp_times': self.thrust_sp.t,
                'sp_data': self.thrust_sp.data()
            })

        if self.torque.t:
            metrics_list.append({
                'metric_name': "Torque",
                'file_prefix': "torque",
                'csv_prefix': "Torque",
                'labels': ["X", "Y", "Z"],
                'ylabels': ('Torque X (norm)', 'Torque Y (norm)', 'Torque Z (norm)'),
                'rms_unit': "",
                'actual_times': self.torque.t,
                'actual_data': self.torque.data(),
                'sp_times': self.torque_sp.t,
                'sp_data': self.torque_sp.data()
            })

        # 1. Export unified CSV
        if test_mode is not None:
            self.export_test_region_to_csv(global_start, global_end, f"data_recording/tmp/{test_mode}_test_region.csv", metrics_list)

        # 2. Generate standard single-run graphs
        for m in metrics_list:
            self.generate_metric_graphs(
                metric_name=m['metric_name'],
                actual_times=m['actual_times'],
                actual_data=m['actual_data'],
                sp_times=m['sp_times'],
                sp_data=m['sp_data'],
                ylabels=m['ylabels'],
                rms_unit=m['rms_unit'],
                file_prefix=m['file_prefix'],
                global_start=global_start,
                global_end=global_end
            )

        # 3. Generate comparison graphs
        self.generate_comparison_graphs(metrics_list)

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