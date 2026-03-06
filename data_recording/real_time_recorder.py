import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleLocalPosition, VehicleStatus, VehicleAttitude, 
                        VehicleAttitudeSetpoint, VehicleRatesSetpoint, VehicleOdometry, 
                        TrajectorySetpoint, OffboardControlMode)
from rclpy.executors import ExternalShutdownException
import matplotlib.pyplot as plt
import time
import math
import numpy as np
import csv
import itertools
import os
import signal

class Series3D:
    def __init__(self):
        self.t, self.x, self.y, self.z = [], [], [], []

    def append(self, t, x, y, z):
        self.t.append(t); self.x.append(x); self.y.append(y); self.z.append(z)

    def data(self):
        return (self.x, self.y, self.z)

class data_recorder(Node):
    def __init__(self):
        super().__init__('local_position_plotter')
        qos = qos_profile_sensor_data

        self.subs = [
            (VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_cb),
            (TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.pos_sp_cb),
            (VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb),
            (VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', self.att_sp_cb),
            (VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', self.rates_sp_cb),
            (VehicleOdometry, '/fmu/out/vehicle_odometry', self.rates_cb),
            (OffboardControlMode, '/fmu/in/offboard_control_mode', self.offboard_cb),
            (VehicleStatus, '/fmu/out/vehicle_status_v2', self.status_cb)
        ]
        for msg_type, topic, callback_function in self.subs:
            self.create_subscription(msg_type, topic, callback_function, qos)

        self.pos, self.pos_sp = Series3D(), Series3D()
        self.att, self.att_sp = Series3D(), Series3D()
        self.rates, self.rates_sp = Series3D(), Series3D()

        self.test_mode_detected = None
        self.test_region_times = []
        self.start_time = None
        self.previous_arming_state = 0
        
        self.get_logger().info('Position Plotter Node started. Subscriptions initialized.')
        self.get_logger().info('Recording data... Press Ctrl+C to stop and generate graphs.')
        
        # Start a 5-second timer to detect if we are running standalone
        self.node_creation_time = time.time()
        self.idle_timeout_timer = self.create_timer(1.0, self.check_idle_timeout)

    def check_idle_timeout(self):
        # If we received data, cancel the timeout so we don't accidentally shut down during a live test
        if len(self.att.t) > 0:
            self.idle_timeout_timer.cancel()
            return
            
        # If 5 seconds have passed with no data, trigger a graceful shutdown by simulating a keyboard interrupt
        if time.time() - self.node_creation_time > 5.0:
            self.get_logger().info("No live data detected. Auto-stopping to regenerate plots from CSV...")
            self.idle_timeout_timer.cancel()
            os.kill(os.getpid(), signal.SIGINT)

    def get_time(self):
        if self.start_time is None: 
            self.start_time = time.time()
        return time.time() - self.start_time

    def q2rpy(self, q):
        roll = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        pitch = math.asin(np.clip(2*(q[0]*q[2]-q[3]*q[1]), -1.0, 1.0))
        yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def pos_cb(self, msg): self.pos.append(self.get_time(), msg.x, msg.y, msg.z)
    def pos_sp_cb(self, msg): self.pos_sp.append(self.get_time(), msg.position[0], msg.position[1], msg.position[2])
    def att_cb(self, msg): self.att.append(self.get_time(), *self.q2rpy(msg.q))
    def att_sp_cb(self, msg): self.att_sp.append(self.get_time(), *self.q2rpy(msg.q_d))
    def rates_cb(self, msg): self.rates.append(self.get_time(), *np.degrees(msg.angular_velocity))
    def rates_sp_cb(self, msg): self.rates_sp.append(self.get_time(), math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw))

    def offboard_cb(self, msg):
        current_time = self.get_time()
        mode_string = "body_rate" if (msg.body_rate and not msg.attitude and not msg.position) else \
                      "attitude" if (msg.attitude and not msg.position) else None
        if mode_string:
            if self.test_mode_detected != mode_string:
                self.get_logger().info(f"{mode_string.replace('_', ' ').title()} mode detected at t={current_time:.2f}s. Tagging test region.")
            self.test_mode_detected = mode_string
            self.test_region_times.append(current_time)

    def status_cb(self, msg):
        if self.previous_arming_state == 2 and msg.arming_state != 2: 
            self.get_logger().info("Disarm detected! Interrupting node to generate plots...")
            raise KeyboardInterrupt
        self.previous_arming_state = msg.arming_state

    def calculate_rms(self, actual_times, actual_data, sp_times, sp_data, test_start, test_end, fraction_of_final_value=0.95):
        if len(actual_times) == 0 or len(sp_times) == 0:
            return None, None, None, None, None

        actual_times_array = np.array(actual_times)
        actual_data_array = np.array(actual_data)
        sp_times_array = np.array(sp_times)
        sp_data_array = np.array(sp_data)
        
        start_bound = max(sp_times_array[0], test_start) if test_start is not None else sp_times_array[0]
        end_bound = min(sp_times_array[-1], test_end) if test_end is not None else sp_times_array[-1]
        
        time_mask = (actual_times_array >= start_bound) & (actual_times_array <= end_bound)
        valid_times = actual_times_array[time_mask]
        valid_data = actual_data_array[time_mask]
        
        if len(valid_times) > 1:
            aligned_setpoint = np.interp(valid_times, sp_times_array, sp_data_array)
            tracking_error = valid_data - aligned_setpoint
            
            total_duration = valid_times[-1] - valid_times[0]
            tracking_rms = np.sqrt(np.trapz(np.square(tracking_error), x=valid_times) / total_duration) if total_duration > 0 else None

            step_range = aligned_setpoint[-1] - valid_data[0]
            tolerance = max((1.0 - fraction_of_final_value) * abs(step_range), 1e-2) if abs(step_range) > 1e-3 else 1e-2
            
            if abs(step_range) > 1e-3:
                outliers = np.where(np.abs(tracking_error) > tolerance)[0]
                steady_state_index = outliers[-1] + 1 if len(outliers) > 0 else 0
                if steady_state_index >= len(valid_times) - 2:
                    steady_state_index = int(len(valid_times) * 0.8)
            else:
                steady_state_index = int(len(valid_times) * 0.8)

            steady_state_times = valid_times[steady_state_index:]
            steady_state_error = tracking_error[steady_state_index:]
            
            steady_state_duration = steady_state_times[-1] - steady_state_times[0] if len(steady_state_times) > 1 else 0
            steady_state_rms = np.sqrt(np.trapz(np.square(steady_state_error), x=steady_state_times) / steady_state_duration) if steady_state_duration > 0 else None
            
            steady_state_start_time = valid_times[steady_state_index] if steady_state_index < len(valid_times) else None
            final_value = np.mean(valid_data[steady_state_index:]) if len(valid_data[steady_state_index:]) > 0 else valid_data[-1]

            return tracking_rms, steady_state_rms, steady_state_start_time, final_value, tolerance
            
        return None, None, None, None, None

    def evaluate_run_metrics(self, actual_times, actual_data, setpoint_times, setpoint_data, test_start, test_end, measurement_unit):
        """Helper to run RMS math loop efficiently for both single and comparison plots."""
        tracking_rms_values = []
        steady_state_rms_values = []
        steady_state_start_times = []
        final_values = []
        tolerance_values = []
        
        for index in range(3):
            tracking_rms, steady_state_rms, steady_state_start, final_value, tolerance = self.calculate_rms(
                actual_times, actual_data[index], setpoint_times, setpoint_data[index], test_start, test_end)
                
            tracking_rms_values.append(f"{tracking_rms:.3f}{measurement_unit}" if tracking_rms is not None else None)
            steady_state_rms_values.append(f"{steady_state_rms:.3f}{measurement_unit}" if steady_state_rms is not None else None)
            steady_state_start_times.append(steady_state_start)
            final_values.append(final_value)
            tolerance_values.append(tolerance)
            
        return tracking_rms_values, steady_state_rms_values, steady_state_start_times, final_values, tolerance_values

    def create_plot(self, title, plot_runs, y_axis_labels, filename):
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(title, fontsize=16)
        
        for index, ax in enumerate(axes):
            rms_text_boxes = []
            for run in plot_runs:
                # SAFE CHECK: Evaluates explicitly to avoid Numpy Truth Value Ambiguity Error
                if run.get('actual_times') is None or len(run['actual_times']) == 0: 
                    continue
                
                plot_color = run.get('color', ['r', 'g', 'b'][index])
                label_prefix = f"{run.get('label_prefix', '')} ".lstrip()
                
                if run.get('sp_times') is not None and len(run['sp_times']) > 0: 
                    ax.plot(run['sp_times'], run['sp_data'][index], color=plot_color, linestyle='--', alpha=0.5, label=f"{label_prefix}Setpoint".strip())
                    
                    if run.get('tolerances') and run['tolerances'][index] is not None:
                        ss_start_time = run.get('rms_starts')[index] if run.get('rms_starts') else None
                        if ss_start_time is not None:
                            sp_times_array = np.array(run['sp_times'])
                            band_mask = sp_times_array >= ss_start_time
                            band_sp_times = sp_times_array[band_mask]
                            setpoint_array_band = np.array(run['sp_data'][index])[band_mask]
                            
                            if len(band_sp_times) > 0:
                                ax.fill_between(band_sp_times, setpoint_array_band - run['tolerances'][index], setpoint_array_band + run['tolerances'][index], 
                                                color=plot_color, alpha=0.1, linewidth=0, label=f"{label_prefix}±{run['tolerances'][index]:.3f} Band".strip())
                
                ax.plot(run['actual_times'], run['actual_data'][index], color=plot_color, linewidth=2, label=f"{label_prefix}Actual".strip())
                
                if run.get('rms_starts') and run['rms_starts'][index] is not None:
                    ax.axvline(x=run['rms_starts'][index], color='black' if not run.get('label_prefix') else plot_color, linestyle=':', alpha=0.7, label=f"{label_prefix}SS Start".strip())
                
                tracking_string = f"Trk RMS: {run['tracking_rms_vals'][index]}" if run.get('tracking_rms_vals') and run['tracking_rms_vals'][index] else ""
                steady_state_string = f"SS RMS: {run['steady_state_rms_vals'][index]}" if run.get('steady_state_rms_vals') and run['steady_state_rms_vals'][index] else ""
                
                if tracking_string or steady_state_string: 
                    rms_text_boxes.append(f"{label_prefix}{tracking_string} | {steady_state_string}".strip(' |'))
            
            ax.set_ylabel(y_axis_labels[index])
            handles, labels = ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=8)
            ax.grid(True)
            if rms_text_boxes: 
                ax.text(1.02, 0.05, "\n".join(rms_text_boxes), transform=ax.transAxes, fontsize=9, bbox=dict(facecolor='white', alpha=0.8), verticalalignment='bottom')
        
        fig.tight_layout(rect=[0, 0, 0.8, 1])
        fig.savefig(filename, dpi=300)
        print(f"[INFO] Saved plot -> {filename}")

    def append_to_summary_table(self, summary_data):
        summary_file = "data_recording/tmp/rms_summary_table.csv"
        file_exists = os.path.isfile(summary_file)
        
        # Explicit headers so column drift is impossible
        headers = [
            "Timestamp", "Control Mode", 
            "Pos X Trk", "Pos Y Trk", "Pos Z Trk",
            "Pos X SS", "Pos Y SS", "Pos Z SS",
            "Pos X Final", "Pos Y Final", "Pos Z Final",
            "Roll Trk", "Pitch Trk", "Yaw Trk",
            "Roll SS", "Pitch SS", "Yaw SS",
            "Roll Final", "Pitch Final", "Yaw Final"
        ]
        
        def format_value(val): return f"{val:.4f}" if val is not None else "N/A"
        
        # Explicit assignments locking everything to the exact proper header
        row = [
            time.strftime("%Y-%m-%d %H:%M:%S"),
            summary_data.get('mode', 'Unknown'),
            format_value(summary_data.get('Position_X_Trk')), format_value(summary_data.get('Position_Y_Trk')), format_value(summary_data.get('Position_Z_Trk')),
            format_value(summary_data.get('Position_X_SS')), format_value(summary_data.get('Position_Y_SS')), format_value(summary_data.get('Position_Z_SS')),
            format_value(summary_data.get('Position_X_Final')), format_value(summary_data.get('Position_Y_Final')), format_value(summary_data.get('Position_Z_Final')),
            format_value(summary_data.get('Attitude_Roll_Trk')), format_value(summary_data.get('Attitude_Pitch_Trk')), format_value(summary_data.get('Attitude_Yaw_Trk')),
            format_value(summary_data.get('Attitude_Roll_SS')), format_value(summary_data.get('Attitude_Pitch_SS')), format_value(summary_data.get('Attitude_Yaw_SS')),
            format_value(summary_data.get('Attitude_Roll_Final')), format_value(summary_data.get('Attitude_Pitch_Final')), format_value(summary_data.get('Attitude_Yaw_Final'))
        ]
        
        with open(summary_file, 'a', newline='') as file_object:
            csv_writer = csv.writer(file_object)
            if not file_exists or os.path.getsize(summary_file) == 0: 
                csv_writer.writerow(headers)
            csv_writer.writerow(row)
            
        print(f"\n{'='*115}\n{'CURRENT RUN RMS SUMMARY (' + str(row[1]) + ' mode)':^115}\n{'='*115}")
        print(f"{'Metric':<15} | {'Tracking RMS (X/R, Y/P, Z/Y)':<30} | {'Steady-State RMS (X/R, Y/P, Z/Y)':<30} | {'Final Value (X/R, Y/P, Z/Y)':<30}\n{'-' * 115}")
        
        # Formatted using strictly explicit indices matching the row assignment
        pos_tracking_string = f"{row[2]}, {row[3]}, {row[4]}"
        pos_steady_state_string  = f"{row[5]}, {row[6]}, {row[7]}"
        pos_final_string = f"{row[8]}, {row[9]}, {row[10]}"
        
        att_tracking_string = f"{row[11]}, {row[12]}, {row[13]}"
        att_steady_state_string  = f"{row[14]}, {row[15]}, {row[16]}"
        att_final_string = f"{row[17]}, {row[18]}, {row[19]}"
        
        print(f"{'Position (m)':<15} | {pos_tracking_string:<30} | {pos_steady_state_string:<30} | {pos_final_string:<30}")
        print(f"{'Attitude (deg)':<15} | {att_tracking_string:<30} | {att_steady_state_string:<30} | {att_final_string:<30}")
        print(f"{'='*115}\n[INFO] Summary appended to: {summary_file}\n")

    def generate_graphs(self):
        print("[INFO] --- Starting Graph Generation ---")
        
        plot_configurations = [
            ('Attitude', 'attitude', 'Attitude', ['Roll', 'Pitch', 'Yaw'], ('Roll (°)', 'Pitch (°)', 'Yaw (°)'), "°", self.att, self.att_sp),
            ('Body Rates', 'rates', 'Rates', ['RollRate', 'PitchRate', 'YawRate'], ('Roll Rate (°/s)', 'Pitch Rate (°/s)', 'Yaw Rate (°/s)'), "°/s", self.rates, self.rates_sp),
            ('Position', 'position', 'Position', ['X', 'Y', 'Z'], ('X (m)', 'Y (m)', 'Z (m)'), "m", self.pos, self.pos_sp)
        ]
        
        # Check if we actually collected live data
        if not self.att.t: 
            print("[INFO] No live data recorded. Skipping new plot generation and CSV export.")
        else:
            test_start = self.test_region_times[0] if self.test_region_times else None
            test_end = self.test_region_times[-1] if self.test_region_times else None
            test_mode = self.test_mode_detected

            if test_mode: 
                self.export_csv(test_start, test_end, f"data_recording/tmp/{test_mode}_test_region.csv", plot_configurations)
            else: 
                print("[WARN] No specific test mode detected via OffboardControlMode. CSV export skipped.")

            summary_data = {'mode': test_mode}

            for metric_name, file_prefix, csv_prefix, axis_labels, y_axis_labels, measurement_unit, actual_container, setpoint_container in plot_configurations:
                if not actual_container.t: 
                    print(f"[WARN] Skipping {metric_name}: No actual data collected.")
                    continue
                    
                print(f"[INFO] Processing {metric_name} metric...")
                
                tracking_rms_values, steady_state_rms_values, steady_state_start_times, final_values, tolerance_values = self.evaluate_run_metrics(
                    actual_container.t, actual_container.data(), setpoint_container.t, setpoint_container.data(), test_start, test_end, measurement_unit)
                
                for index, label in enumerate(axis_labels):
                    # Safely parse strings to floats without throwing an exception if the RMS value is None
                    parsed_tracking = float(tracking_rms_values[index].replace(measurement_unit,'')) if tracking_rms_values[index] else None
                    parsed_steady_state = float(steady_state_rms_values[index].replace(measurement_unit,'')) if steady_state_rms_values[index] else None
                    
                    summary_data.update({
                        f"{metric_name}_{label}_Trk": parsed_tracking, 
                        f"{metric_name}_{label}_SS": parsed_steady_state, 
                        f"{metric_name}_{label}_Final": final_values[index]
                    })

                run_base_configuration = {
                    'tracking_rms_vals': tracking_rms_values, 
                    'steady_state_rms_vals': steady_state_rms_values, 
                    'rms_starts': steady_state_start_times, 
                    'tolerances': tolerance_values
                }
                
                self.create_plot(f"UAV {metric_name} vs Time", [{**run_base_configuration, 'actual_times': actual_container.t, 'actual_data': actual_container.data(), 'sp_times': setpoint_container.t, 'sp_data': setpoint_container.data()}], y_axis_labels, f"data_recording/tmp/run_{file_prefix}.png")
                
                if test_start is not None:
                    mask_actual = (np.array(actual_container.t) >= test_start) & (np.array(actual_container.t) <= test_end)
                    mask_setpoint = (np.array(setpoint_container.t) >= test_start) & (np.array(setpoint_container.t) <= test_end)
                    
                    run_cropped_configuration = {
                        **run_base_configuration, 
                        'actual_times': np.array(actual_container.t)[mask_actual] - test_start, 
                        'actual_data': [np.array(data)[mask_actual] for data in actual_container.data()], 
                        'sp_times': np.array(setpoint_container.t)[mask_setpoint] - test_start, 
                        'sp_data': [np.array(data)[mask_setpoint] for data in setpoint_container.data()],
                        'rms_starts': [start_time - test_start if start_time else None for start_time in steady_state_start_times]
                    }
                    
                    self.create_plot(f"UAV {metric_name} (Test Region)", [run_cropped_configuration], y_axis_labels, f"data_recording/tmp/run_{file_prefix}_cropped.png")

            # Output the tabular summary to both terminal and CSV
            self.append_to_summary_table(summary_data)
            
        # This will ALWAYS run, allowing regeneration of CSV plots if no live data is detected.
        self.generate_comparison_graphs(plot_configurations)
        print("[INFO] --- Graph Generation Complete ---")

    def export_csv(self, test_start, test_end, filename, plot_configurations):
        print(f"[INFO] Exporting unified CSV -> {filename}")
        csv_columns = {}
        for metric_name, file_prefix, csv_prefix, axis_labels, y_axis_labels, measurement_unit, actual_container, setpoint_container in plot_configurations:
            if not actual_container.t: 
                continue
                
            valid_time_mask = (np.array(actual_container.t) >= test_start) & (np.array(actual_container.t) <= test_end)
            csv_columns[f'{csv_prefix}_Time'] = np.array(actual_container.t)[valid_time_mask] - test_start
            
            for index, label in enumerate(axis_labels): 
                csv_columns[f'{csv_prefix}_{label}'] = np.array(actual_container.data()[index])[valid_time_mask]
                
            if setpoint_container.t:
                setpoint_time_mask = (np.array(setpoint_container.t) >= test_start) & (np.array(setpoint_container.t) <= test_end)
                csv_columns[f'{csv_prefix}_SP_Time'] = np.array(setpoint_container.t)[setpoint_time_mask] - test_start
                for index, label in enumerate(axis_labels): 
                    csv_columns[f'{csv_prefix}_SP_{label}'] = np.array(setpoint_container.data()[index])[setpoint_time_mask]
                    
        with open(filename, 'w') as file_object:
            csv_writer = csv.writer(file_object)
            csv_writer.writerow(csv_columns.keys())
            csv_writer.writerows(itertools.zip_longest(*csv_columns.values(), fillvalue=''))

    def generate_comparison_graphs(self, plot_configurations):
        attitude_csv_file = "data_recording/tmp/attitude_test_region.csv"
        body_rate_csv_file = "data_recording/tmp/body_rate_test_region.csv"
        
        if not (os.path.exists(attitude_csv_file) and os.path.exists(body_rate_csv_file)): 
            print("[INFO] Comparison CSVs not found. Skipping overlay plots.")
            return
            
        print("[INFO] Found both Attitude and Body Rate CSVs. Generating overlay plots...")
        
        def load_comparison_csv(filepath):
            with open(filepath, 'r') as file_object:
                csv_reader = csv.reader(file_object)
                headers = next(csv_reader)
                data_dictionary = {header: [] for header in headers}
                for row in csv_reader:
                    for index, value in enumerate(row):
                        if value.strip(): 
                            data_dictionary[headers[index]].append(float(value))
            return {key: np.array(value) for key, value in data_dictionary.items()}
            
        attitude_data = load_comparison_csv(attitude_csv_file)
        body_rate_data = load_comparison_csv(body_rate_csv_file)
        
        for metric_name, file_prefix, csv_prefix, axis_labels, y_axis_labels, measurement_unit, actual_container, setpoint_container in plot_configurations:
            comparison_runs = []
            
            for data_dictionary, plot_color, label_prefix in [(attitude_data, 'blue', 'Att Mode'), (body_rate_data, 'orange', 'BR Mode')]:
                actual_times_list = data_dictionary.get(f'{csv_prefix}_Time', [])
                if len(actual_times_list) == 0: 
                    continue
                
                actual_data_list = [data_dictionary.get(f'{csv_prefix}_{label}', []) for label in axis_labels]
                setpoint_times_list = data_dictionary.get(f'{csv_prefix}_SP_Time', [])
                setpoint_data_list = [data_dictionary.get(f'{csv_prefix}_SP_{label}', []) for label in axis_labels]
                
                tracking_rms_values, steady_state_rms_values, steady_state_start_times, final_values, tolerance_values = self.evaluate_run_metrics(
                    actual_times_list, actual_data_list, setpoint_times_list, setpoint_data_list, 0, None, measurement_unit)
                
                comparison_runs.append({
                    'actual_times': actual_times_list, 
                    'actual_data': actual_data_list, 
                    'sp_times': setpoint_times_list, 
                    'sp_data': setpoint_data_list, 
                    'color': plot_color, 
                    'label_prefix': label_prefix, 
                    'tracking_rms_vals': tracking_rms_values, 
                    'steady_state_rms_vals': steady_state_rms_values
                    # Note: 'rms_starts' and 'tolerances' omitted here to keep the overlay plots clean
                })
            
            if comparison_runs: 
                self.create_plot(f"UAV {metric_name} Comparison", comparison_runs, y_axis_labels, f"data_recording/tmp/comparison_{csv_prefix.lower()}.png")

def main():
    rclpy.init()
    node = data_recorder()
    try: 
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): 
        pass
    except Exception as error:
        if "context is not valid" not in str(error): 
            raise
    finally: 
        print("\n[INFO] Recording stopped. Initiating plot export...")
        node.generate_graphs()
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__': main()