import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import Stanley
import PID
import LQR
import MPC
from cubic_spline import calc_spline_course
import draw

def simulate_path(cx, cy, cyaw, ck, path_name, goal_idx=None, sp_profile=None):
    dt = MPC.P.dt
    target_speed = MPC.P.target_speed

    Stanley.C.dt = dt
    PID.C.dt = dt
    LQR.ts = dt

    Stanley.C.dref = MPC.P.dist_stop
    PID.C.d_stop = MPC.P.dist_stop
    MPC.P.speed_stop = MPC.P.speed_max

    # 起点
    x0, y0, yaw0 = cx[0], cy[0], cyaw[0]

    # 增大最近点窗口
    MPC.P.N_IND = 200

    stan_node = Stanley.Node(x0, y0, yaw0, v=0.0)
    stan_traj = Stanley.Trajectory(cx, cy, cyaw)
    pid_node = PID.Node(x0, y0, yaw0, v=0.0)
    pid_traj = PID.Trajectory(cx, cy, cyaw)
    pid_controller = PID.ControllerPurePID()
    lqr_ref_traj = LQR.TrajectoryAnalyzer(cx, cy, cyaw, ck)
    lqr_vehicle = LQR.VehicleState(x=x0, y=y0, yaw=yaw0, v=0.0, gear=LQR.Gear.GEAR_DRIVE)
    lqr_lat_controller = LQR.LatController()
    lqr_lon_controller = LQR.LonController()
    mpc_ref_path = MPC.PATH(cx, cy, cyaw, ck)
    mpc_sp = sp_profile if sp_profile is not None else MPC.calc_speed_profile(cx, cy, cyaw, target_speed)
    mpc_node = MPC.Node(x=x0, y=y0, yaw=yaw0, v=0.0)
    delta_opt, a_opt = None, None
    delta_exc, a_exc = 0.0, 0.0

    # 终点
    if goal_idx is not None:
        goal_x, goal_y = cx[goal_idx], cy[goal_idx]
    else:
        goal_x, goal_y = cx[-1], cy[-1]

    time = 0.0
    max_time = 100.0
    traj_stan_x, traj_stan_y = [], []
    traj_pid_x, traj_pid_y = [], []
    traj_lqr_x, traj_lqr_y = [], []
    traj_mpc_x, traj_mpc_y = [], []
    stan_errors, pid_errors, lqr_errors, mpc_errors = [], [], [], []
    stan_speed_errs, pid_speed_errs, lqr_speed_errs, mpc_speed_errs = [], [], [], []
    stan_done = pid_done = lqr_done = mpc_done = False
    stan_finish_t = pid_finish_t = lqr_finish_t = mpc_finish_t = None
    yaw_old_stan = stan_node.yaw
    yaw_old_pid = pid_node.yaw
    yaw_old_mpc = mpc_node.yaw

    plt.figure()
    plt.axis("equal")
    plt.ion()

    while time < max_time:
        if stan_done and pid_done and lqr_done and mpc_done:
            break
        # Stanley
        if not stan_done:
            stan_delta, _ = Stanley.front_wheel_feedback_control(stan_node, stan_traj)
            dist = math.hypot(stan_node.x - goal_x, stan_node.y - goal_y)
            stan_acc = Stanley.pid_control(target_speed, stan_node.v, dist)
            stan_node.update(stan_acc, stan_delta)
            theta_e, e_f, _ = stan_traj.calc_theta_e_and_ef(stan_node)
            stan_errors.append(abs(float(e_f)))
            stan_speed_errs.append(target_speed - stan_node.v)
            traj_stan_x.append(stan_node.x)
            traj_stan_y.append(stan_node.y)
            if dist <= MPC.P.dist_stop:
                stan_done = True
                stan_finish_t = time
        # PID
        if not pid_done:
            pid_a, pid_delta, theta_e_pid, e_f_pid = pid_controller.control(pid_node, pid_traj)
            pid_node.update(pid_a, pid_delta)
            dist = math.hypot(pid_node.x - goal_x, pid_node.y - goal_y)
            pid_errors.append(abs(e_f_pid))
            pid_speed_errs.append(target_speed - pid_node.v)
            traj_pid_x.append(pid_node.x)
            traj_pid_y.append(pid_node.y)
            if dist <= MPC.P.dist_stop:
                pid_done = True
                pid_finish_t = time
        # LQR
        if not lqr_done:
            dist = math.hypot(lqr_vehicle.x - goal_x, lqr_vehicle.y - goal_y)
            tgt_speed = target_speed
            steer_cmd, theta_e_lqr, e_cg = lqr_lat_controller.ComputeControlCommand(lqr_vehicle, lqr_ref_traj)
            accel_cmd = LQR.LonController.ComputeControlCommand(tgt_speed, lqr_vehicle, dist)
            lqr_vehicle.UpdateVehicleState(steer_cmd, accel_cmd, e_cg, theta_e_lqr, gear=LQR.Gear.GEAR_DRIVE)
            lqr_errors.append(abs(e_cg))
            lqr_speed_errs.append(target_speed - lqr_vehicle.v)
            traj_lqr_x.append(lqr_vehicle.x)
            traj_lqr_y.append(lqr_vehicle.y)
            if dist <= MPC.P.dist_stop:
                lqr_done = True
                lqr_finish_t = time
        # MPC
        if not mpc_done:
            z_ref, target_ind = MPC.calc_ref_trajectory_in_T_step(mpc_node, mpc_ref_path, mpc_sp)
            z0 = [mpc_node.x, mpc_node.y, mpc_node.v, mpc_node.yaw]
            a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = MPC.linear_mpc_control(z_ref, z0, a_opt, delta_opt)
            if delta_opt is not None:
                delta_exc, a_exc = delta_opt[0], a_opt[0]
            mpc_node.update(a_exc, delta_exc, 1.0)
            dist = math.hypot(mpc_node.x - goal_x, mpc_node.y - goal_y)
            _, er = mpc_ref_path.nearest_index(mpc_node)
            mpc_errors.append(abs(er))
            mpc_speed_errs.append(target_speed - mpc_node.v)
            traj_mpc_x.append(mpc_node.x)
            traj_mpc_y.append(mpc_node.y)
            if dist <= MPC.P.dist_stop:
                mpc_done = True
                mpc_finish_t = time

        # Visualization
        plt.cla()
        plt.plot(cx, cy, color='gray', linewidth=2, label='Path')
        plt.plot(traj_stan_x, traj_stan_y, color='red', linewidth=2, label='Stanley')
        plt.plot(traj_pid_x, traj_pid_y, color='blue', linewidth=2, label='PID')
        plt.plot(traj_lqr_x, traj_lqr_y, color='green', linewidth=2, label='LQR')
        plt.plot(traj_mpc_x, traj_mpc_y, color='purple', linewidth=2, label='MPC')
        plt.plot(goal_x, goal_y, 'mo', markersize=10, label='Goal')
        steer_vis_stan = 0.0
        if stan_node.v != 0:
            steer_vis_stan = -math.atan(Stanley.C.WB * ((stan_node.yaw - yaw_old_stan) / (stan_node.v * dt + 1e-6)))
        yaw_old_stan = stan_node.yaw
        draw.draw_car(stan_node.x, stan_node.y, stan_node.yaw, steer_vis_stan, Stanley.C, color='red')
        steer_vis_pid = 0.0
        if pid_node.v != 0:
            steer_vis_pid = -math.atan(PID.C.WB * ((pid_node.yaw - yaw_old_pid) / (pid_node.v * dt + 1e-6)))
        yaw_old_pid = pid_node.yaw
        draw.draw_car(pid_node.x, pid_node.y, pid_node.yaw, steer_vis_pid, PID.C, color='blue')
        draw.draw_car(lqr_vehicle.x, lqr_vehicle.y, lqr_vehicle.yaw, -lqr_vehicle.steer, Stanley.C, color='green')
        steer_vis_mpc = 0.0
        if mpc_node.v != 0:
            steer_vis_mpc = -math.atan(MPC.P.WB * ((mpc_node.yaw - yaw_old_mpc) / (mpc_node.v * dt + 1e-6)))
        yaw_old_mpc = mpc_node.yaw
        draw.draw_car(mpc_node.x, mpc_node.y, mpc_node.yaw, steer_vis_mpc, MPC.P, color='purple')
        plt.axis("equal")
        plt.title(f"Path Tracking - {path_name} Path")
        plt.legend()
        plt.pause(0.001)
        time += dt

    plt.ioff()
    plt.show()

    def average_lateral_error(errors):
        return float(np.mean(errors)) if errors else 0.0
    def max_lateral_error(errors):
        return float(np.max(errors)) if errors else 0.0
    def steady_state_speed_error(speed_errs, finish_t):
        if finish_t is None or not speed_errs:
            return None
        n_last = int(5.0 / dt)
        segment = speed_errs[-n_last:] if len(speed_errs) >= n_last else speed_errs
        return float(np.mean(segment))

    if stan_finish_t is None: stan_finish_t = time
    if pid_finish_t is None: pid_finish_t = time
    if lqr_finish_t is None: lqr_finish_t = time
    if mpc_finish_t is None: mpc_finish_t = time

    results = []
    results.append({
        "Path": path_name,
        "Controller": "Stanley",
        "Avg Lat Err (m)": average_lateral_error(stan_errors),
        "Max Lat Err (m)": max_lateral_error(stan_errors),
        "Completion Time (s)": stan_finish_t,
        "Steady Speed Err (m/s)": steady_state_speed_error(stan_speed_errs, stan_finish_t)
    })
    results.append({
        "Path": path_name,
        "Controller": "PID",
        "Avg Lat Err (m)": average_lateral_error(pid_errors),
        "Max Lat Err (m)": max_lateral_error(pid_errors),
        "Completion Time (s)": pid_finish_t,
        "Steady Speed Err (m/s)": steady_state_speed_error(pid_speed_errs, pid_finish_t)
    })
    results.append({
        "Path": path_name,
        "Controller": "LQR",
        "Avg Lat Err (m)": average_lateral_error(lqr_errors),
        "Max Lat Err (m)": max_lateral_error(lqr_errors),
        "Completion Time (s)": lqr_finish_t,
        "Steady Speed Err (m/s)": steady_state_speed_error(lqr_speed_errs, lqr_finish_t)
    })
    results.append({
        "Path": path_name,
        "Controller": "MPC",
        "Avg Lat Err (m)": average_lateral_error(mpc_errors),
        "Max Lat Err (m)": max_lateral_error(mpc_errors),
        "Completion Time (s)": mpc_finish_t,
        "Steady Speed Err (m/s)": steady_state_speed_error(mpc_speed_errs, mpc_finish_t)
    })
    return results

if __name__ == "__main__":
    # 1. 直线任务
    ax_straight = np.arange(0, 50, 0.5)
    ay_straight = np.zeros_like(ax_straight)
    cx_s, cy_s, cyaw_s, ck_s, s_s = calc_spline_course(ax_straight, ay_straight, ds=0.1)

    # 2. 曲线任务
    ax_curve = np.arange(0, 50, 0.5)
    ay_curve = [math.sin(ix / 5.0) * ix / 2.0 for ix in ax_curve]
    cx_c, cy_c, cyaw_c, ck_c, s_c = calc_spline_course(ax_curve, ay_curve, ds=0.1)

    # 3. 圆任务
    R = 20.0
    N = 200
    theta = np.linspace(0, 2*np.pi, N, endpoint=False)
    cx_o = R * np.cos(theta)
    cy_o = R * np.sin(theta)
    cyaw_o = theta + np.pi/2
    ck_o = [1.0/R]*N

    # 圆的3/4终点与速度剖面
    goal_idx = int(N * 0.75)
    goal_x, goal_y = cx_o[goal_idx], cy_o[goal_idx]
    sp_o = [MPC.P.target_speed] * N
    N_drop = min(30, N)
    for i in range(N_drop):
        sp_o[(goal_idx - i) % N] = MPC.P.target_speed * (N_drop - i) / N_drop
    sp_o[goal_idx] = 0.0

    results_all = []
    results_all += simulate_path(cx_s, cy_s, cyaw_s, ck_s, "Straight")
    results_all += simulate_path(cx_c, cy_c, cyaw_c, ck_c, "Curved")
    results_all += simulate_path(cx_o, cy_o, cyaw_o, ck_o, "Circle_3_4", goal_idx=goal_idx, sp_profile=sp_o)

    df = pd.DataFrame(results_all)
    df["Avg Lat Err (m)"] = df["Avg Lat Err (m)"].apply(lambda x: f"{x:.3f}" if x is not None else "N/A")
    df["Max Lat Err (m)"] = df["Max Lat Err (m)"].apply(lambda x: f"{x:.3f}" if x is not None else "N/A")
    df["Completion Time (s)"] = df["Completion Time (s)"].apply(lambda x: f"{x:.1f}" if x is not None else "N/A")
    df["Steady Speed Err (m/s)"] = df["Steady Speed Err (m/s)"].apply(lambda x: f"{x:.3f}" if x is not None else "N/A")
    print(df.to_string(index=False))
