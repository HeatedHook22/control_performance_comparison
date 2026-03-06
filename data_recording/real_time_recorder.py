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
            (VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb)
        ]
        for msg_type, topic, cb in self.subs:
            self.create_subscription(msg_type, topic, cb, qos)

        self.pos, self.pos_sp = Series3D(), Series3D()
        self.att, self.att_sp = Series3D(), Series3D()
        self.rates, self.rates_sp = Series3D(), Series3D()

        self.test_mode_detected, self.test_region_times = None, []
        self.start_time, self.previous_arming_state = None, 0
        
        self.get_logger().info('Position Plotter Node started. Subscriptions initialized.')
        self.get_logger().info('Recording data... Press Ctrl+C to stop and generate graphs.')

    def get_time(self):
        if self.start_time is None: self.start_time = time.time()
        return time.time() - self.start_time

    def q2rpy(self, q):
        r = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))
        p = math.asin(np.clip(2*(q[0]*q[2]-q[3]*q[1]), -1.0, 1.0))
        y = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))
        return math.degrees(r), math.degrees(p), math.degrees(y)

    # Callbacks
    def pos_cb(self, msg): self.pos.append(self.get_time(), msg.x, msg.y, msg.z)
    def pos_sp_cb(self, msg): self.pos_sp.append(self.get_time(), msg.position[0], msg.position[1], msg.position[2])
    def att_cb(self, msg): self.att.append(self.get_time(), *self.q2rpy(msg.q))
    def att_sp_cb(self, msg): self.att_sp.append(self.get_time(), *self.q2rpy(msg.q_d))
    def rates_cb(self, msg): self.rates.append(self.get_time(), *np.degrees(msg.angular_velocity))
    def rates_sp_cb(self, msg): self.rates_sp.append(self.get_time(), math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw))

    def offboard_cb(self, msg):
        t = self.get_time()
        if msg.body_rate and not msg.attitude and not msg.position:
            if self.test_mode_detected != "body_rate":
                self.get_logger().info(f"Body Rate mode detected at t={t:.2f}s. Tagging test region.")
            self.test_mode_detected = "body_rate"; self.test_region_times.append(t)
        elif msg.attitude and not msg.position:
            if self.test_mode_detected != "attitude":
                self.get_logger().info(f"Attitude mode detected at t={t:.2f}s. Tagging test region.")
            self.test_mode_detected = "attitude"; self.test_region_times.append(t)

    def status_cb(self, msg):
        if self.previous_arming_state == 2 and msg.arming_state != 2: 
            self.get_logger().info("Disarm detected! Interrupting node to generate plots...")
            raise KeyboardInterrupt
        self.previous_arming_state = msg.arming_state

    def calculate_rms(self, actual_times, actual_data, sp_times, sp_data, test_start, test_end, fraction_of_final_value = 0.95):
        actual_times, actual_data = np.array(actual_times), np.array(actual_data)
        sp_times, sp_data = np.array(sp_times), np.array(sp_data)
        sb = max(sp_times[0], test_start) if test_start is not None else sp_times[0]
        eb = min(sp_times[-1], test_end) if test_end is not None else sp_times[-1]
        mask = (actual_times >= sb) & (actual_times <= eb)
        v_t, v_d = actual_times[mask], actual_data[mask]
        if len(v_t) > 1:
            a_sp = np.interp(v_t, sp_times, sp_data)
            err = v_d - a_sp
            step_range = a_sp[-1] - v_d[0]
            idx = 0
            if abs(step_range) > 1e-3:
                th = v_d[0] + fraction_of_final_value * step_range
                cr = np.where(v_d >= th)[0] if step_range > 0 else np.where(v_d <= th)[0]
                if len(cr) > 0: idx = cr[0]
            ss_t, ss_e = v_t[idx:], err[idx:]
            T = ss_t[-1] - ss_t[0]
            if T > 0: return np.sqrt(np.trapz(np.square(ss_e), x=ss_t)/T), v_t[idx]
        return None, None

    def create_plot(self, title, plot_runs, ylabels, filename):
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(title, fontsize=16)
        for i, ax in enumerate(axes):
            rms_texts = []
            for run in plot_runs:
                # SAFE CHECK: Evaluates explicitly to avoid Numpy Truth Value Ambiguity Error
                if run.get('actual_times') is None or len(run['actual_times']) == 0: 
                    continue
                
                c = run.get('color', ['r', 'g', 'b'][i])
                pref = f"{run.get('label_prefix', '')} ".lstrip()
                
                # SAFE CHECK: For Setpoint arrays
                if run.get('sp_times') is not None and len(run['sp_times']) > 0: 
                    ax.plot(run['sp_times'], run['sp_data'][i], color=c, linestyle='--', alpha=0.5, label=f"{pref}Setpoint".strip())
                
                ax.plot(run['actual_times'], run['actual_data'][i], color=c, linewidth=2, label=f"{pref}Actual".strip())
                
                if run.get('rms_starts') and run['rms_starts'][i] is not None:
                    ax.axvline(x=run['rms_starts'][i], color='black' if not run.get('label_prefix') else c, linestyle=':', alpha=0.7, label=f"{pref}RMS Start".strip())
                
                if run.get('rms_vals') and run['rms_vals'][i] is not None:
                    rms_texts.append(f"{pref}RMS: {run['rms_vals'][i]}".strip())
            
            ax.set_ylabel(ylabels[i])
            handles, labels = ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), loc="upper left", bbox_to_anchor=(1.02, 1), fontsize=8)
            ax.grid(True)
            if rms_texts: 
                ax.text(1.02, 0.05, "\n".join(rms_texts), transform=ax.transAxes, fontsize=9, bbox=dict(facecolor='white', alpha=0.8), verticalalignment='bottom')
        
        fig.tight_layout(rect=[0, 0, 0.8, 1])
        fig.savefig(filename, dpi=300)
        self.get_logger().info(f"Saved plot -> {filename}")

    def generate_graphs(self):
        self.get_logger().info("--- Starting Graph Generation ---")
        if not self.att.t: 
            self.get_logger().warn("No attitude data recorded. Cannot generate graphs.")
            return
            
        start, end, mode = (self.test_region_times[0], self.test_region_times[-1], self.test_mode_detected) if self.test_region_times else (None, None, None)
        configs = [
            ('Attitude', 'attitude', 'Attitude', ['Roll', 'Pitch', 'Yaw'], ('Roll (°)', 'Pitch (°)', 'Yaw (°)'), "°", self.att, self.att_sp),
            ('Body Rates', 'rates', 'Rates', ['RollRate', 'PitchRate', 'YawRate'], ('Roll Rate (°/s)', 'Pitch Rate (°/s)', 'Yaw Rate (°/s)'), "°/s", self.rates, self.rates_sp),
            ('Position', 'position', 'Position', ['X', 'Y', 'Z'], ('X (m)', 'Y (m)', 'Z (m)'), "m", self.pos, self.pos_sp)
        ]
        
        if mode: 
            self.export_csv(start, end, f"data_recording/tmp/{mode}_test_region.csv", configs)
        else:
            self.get_logger().warn("No specific test mode detected via OffboardControlMode. CSV export skipped.")

        for name, f_pref, c_pref, labels, ylabs, unit, act, sp in configs:
            if not act.t: 
                self.get_logger().warn(f"Skipping {name}: No actual data collected.")
                continue
                
            self.get_logger().info(f"Processing {name} metric...")
            rms_v, rms_s = [], []
            for i in range(3):
                v, st = self.calculate_rms(act.t, act.data()[i], sp.t, sp.data()[i], start, end)
                rms_v.append(f"{v:.3f}{unit}" if v else None); rms_s.append(st)
            
            # Full Plot
            run_f = {'actual_times': act.t, 'actual_data': act.data(), 'sp_times': sp.t, 'sp_data': sp.data(), 'rms_vals': rms_v, 'rms_starts': rms_s}
            self.create_plot(f"UAV {name} vs Time", [run_f], ylabs, f"data_recording/tmp/run_{f_pref}.png")
            
            # Cropped Plot (Test Region Only)
            if start is not None:
                m_a = (np.array(act.t) >= start) & (np.array(act.t) <= end)
                m_s = (np.array(sp.t) >= start) & (np.array(sp.t) <= end)
                c_act_t = np.array(act.t)[m_a] - start
                c_sp_t = np.array(sp.t)[m_s] - start
                c_rms_s = [s - start if s else None for s in rms_s]
                run_c = {'actual_times': c_act_t, 'actual_data': [np.array(d)[m_a] for d in act.data()], 
                         'sp_times': c_sp_t, 'sp_data': [np.array(d)[m_s] for d in sp.data()], 
                         'rms_vals': rms_v, 'rms_starts': c_rms_s}
                self.create_plot(f"UAV {name} (Test Region)", [run_c], ylabs, f"data_recording/tmp/run_{f_pref}_cropped.png")

        self.generate_comparison_graphs(configs)
        self.get_logger().info("--- Graph Generation Complete ---")

    def export_csv(self, start, end, filename, configs):
        self.get_logger().info(f"Exporting unified CSV -> {filename}")
        cols = {}
        for _, _, c_pref, labels, _, _, act, sp in configs:
            if not act.t: continue
            t_arr = np.array(act.t); mask = (t_arr >= start) & (t_arr <= end)
            cols[f'{c_pref}_Time'] = t_arr[mask] - start
            for i, l in enumerate(labels): cols[f'{c_pref}_{l}'] = np.array(act.data()[i])[mask]
            if sp.t:
                spt = np.array(sp.t); sm = (spt >= start) & (spt <= end)
                cols[f'{c_pref}_SP_Time'] = spt[sm] - start
                for i, l in enumerate(labels): cols[f'{c_pref}_SP_{l}'] = np.array(sp.data()[i])[sm]
        with open(filename, 'w') as f:
            w = csv.writer(f); w.writerow(cols.keys()); w.writerows(itertools.zip_longest(*cols.values(), fillvalue=''))

    def generate_comparison_graphs(self, configs):
        att_file, br_file = "data_recording/tmp/attitude_test_region.csv", "data_recording/tmp/body_rate_test_region.csv"
        if not (os.path.exists(att_file) and os.path.exists(br_file)): 
            self.get_logger().info("Comparison CSVs not found. Skipping overlay plots.")
            return
            
        self.get_logger().info("Found both Attitude and Body Rate CSVs. Generating overlay plots...")
        
        def load_csv(path):
            d = {}
            with open(path, 'r') as f:
                r = csv.reader(f); head = next(r)
                for h in head: d[h] = []
                for row in r:
                    for i, v in enumerate(row):
                        if v.strip(): d[head[i]].append(float(v))
            return {k: np.array(v) for k, v in d.items()}
            
        a_data, b_data = load_csv(att_file), load_csv(br_file)
        
        for name, _, c_pref, labels, ylabs, unit, _, _ in configs:
            runs = []
            for d, col, pref in [(a_data, 'blue', 'Att Mode'), (b_data, 'orange', 'BR Mode')]:
                t = d.get(f'{c_pref}_Time', [])
                if len(t) == 0: continue
                data = [d.get(f'{c_pref}_{l}', []) for l in labels]
                spt = d.get(f'{c_pref}_SP_Time', []); spd = [d.get(f'{c_pref}_SP_{l}', []) for l in labels]
                rms_v, rms_s = [], []
                for i in range(3):
                    v, st = self.calculate_rms(t, data[i], spt, spd[i], 0, None)
                    rms_v.append(f"{v:.3f}{unit}" if v else None); rms_s.append(st)
                runs.append({'actual_times': t, 'actual_data': data, 'sp_times': spt, 'sp_data': spd, 'color': col, 'label_prefix': pref, 'rms_vals': rms_v, 'rms_starts': rms_s})
            
            if runs: 
                self.create_plot(f"UAV {name} Comparison", runs, ylabs, f"data_recording/tmp/comparison_{c_pref.lower()}.png")

def main():
    rclpy.init()
    node = data_recorder()
    try: 
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): 
        node.get_logger().info("Recording stopped. Initiating plot export...")
        node.generate_graphs()
    finally: 
        rclpy.shutdown()

if __name__ == '__main__': main()