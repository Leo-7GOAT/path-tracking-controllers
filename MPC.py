import os
import sys
import math
import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import draw as draw
import CurvesGenerator.reeds_shepp as rs
import CurvesGenerator.cubic_spline as cs


class P:
    # System config
    NX = 4  # state vector: z = [x, y, v, phi]
    NU = 2  # input vector: u = [acceleration, steer]
    T = 6  # finite time horizon length

    # MPC config
    Q = np.diag([2.0, 2.0, 1.0, 1.0])  # penalty for states
    Qf = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for end state
    R = np.diag([0.01, 0.1])  # penalty for inputs
    Rd = np.diag([0.01, 0.1])  # penalty for change of inputs

    dist_stop = 1.5  # stop permitted when dist to goal < dist_stop
    speed_stop = 0.5 / 3.6  # stop permitted when speed < speed_stop
    time_max = 500.0  # max simulation time
    iter_max = 5  # max iteration
    target_speed = 10.0 / 3.6  # target speed
    N_IND = 10  # search index number
    dt = 0.2  # time step
    d_dist = 1.0  # dist step
    du_res = 0.1  # threshold for stopping iteration

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width

    steer_max = np.deg2rad(45.0)  # max steering angle [rad]
    steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    speed_max = 55.0 / 3.6  # maximum speed [m/s]
    speed_min = -20.0 / 3.6  # minimum speed [m/s]
    acceleration_max = 1.0  # maximum acceleration [m/s2]


class Node:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * P.dt
        self.y += self.v * math.sin(self.yaw) * P.dt
        self.yaw += self.v / P.WB * math.tan(delta) * P.dt
        self.direct = direct
        self.v += self.direct * a * P.dt
        self.v = self.limit_speed(self.v)

    @staticmethod
    def limit_input_delta(delta):
        if delta >= P.steer_max:
            return P.steer_max

        if delta <= -P.steer_max:
            return -P.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= P.speed_max:
            return P.speed_max

        if v <= P.speed_min:
            return P.speed_min

        return v


class PATH:
    def __init__(self, cx, cy, cyaw, ck):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.length = len(cx)
        self.ind_old = 0

    def nearest_index(self, node):
        """
        Calc index of the nearest node in N steps (supports closed loop/circle).
        :param node: current vehicle node
        :return: nearest index, lateral distance to ref point
        """
        N = self.length
        window = P.N_IND

        # 滑动窗口两端周期取模，保证闭环不跳变
        search_inds = [(self.ind_old + i) % N for i in range(window)]

        dx = [node.x - self.cx[i] for i in search_inds]
        dy = [node.y - self.cy[i] for i in search_inds]
        dist = np.hypot(dx, dy)

        ind_in_N = int(np.argmin(dist))
        ind = search_inds[ind_in_N]
        self.ind_old = ind  # 保持索引历史，防止突变

        rear_axle_vec_rot_90 = np.array([[math.cos(node.yaw + math.pi / 2.0)],
                                         [math.sin(node.yaw + math.pi / 2.0)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        er = er[0][0]

        return ind, er


def calc_ref_trajectory_in_T_step(node, ref_path, sp):
    """
    calc referent trajectory in T steps: [x, y, v, yaw]
    using the current velocity, calc the T points along the reference path
    :param node: current information
    :param ref_path: reference path: [x, y, yaw]
    :param sp: speed profile (designed speed strategy)
    :return: reference trajectory
    """

    z_ref = np.zeros((P.NX, P.T + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(node)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = sp[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0

    for i in range(1, P.T + 1):
        dist_move += abs(node.v) * P.dt
        ind_move = int(round(dist_move / P.d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = sp[index]
        z_ref[3, i] = ref_path.cyaw[index]

    return z_ref, ind


def linear_mpc_control(z_ref, z0, a_old, delta_old):
    """
    linear mpc controller
    :param z_ref: reference trajectory in T steps
    :param z0: initial state vector
    :param a_old: acceleration of T steps of last time
    :param delta_old: delta of T steps of last time
    :return: acceleration and delta strategy based on current information
    """

    if a_old is None or delta_old is None:
        a_old = [0.0] * P.T
        delta_old = [0.0] * P.T

    x, y, yaw, v = None, None, None, None

    for k in range(P.iter_max):
        z_bar = predict_states_in_T_step(z0, a_old, delta_old, z_ref)
        a_rec, delta_rec = a_old[:], delta_old[:]
        a_old, delta_old, x, y, yaw, v = solve_linear_mpc(z_ref, z_bar, z0, delta_old)

        du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
        du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

        if max(du_a_max, du_d_max) < P.du_res:
            break

    return a_old, delta_old, x, y, yaw, v


def predict_states_in_T_step(z0, a, delta, z_ref):
    """
    given the current state, using the acceleration and delta strategy of last time,
    predict the states of vehicle in T steps.
    :param z0: initial state
    :param a: acceleration strategy of last time
    :param delta: delta strategy of last time
    :param z_ref: reference trajectory
    :return: predict states in T steps (z_bar, used for calc linear motion model)
    """

    z_bar = z_ref * 0.0

    for i in range(P.NX):
        z_bar[i, 0] = z0[i]

    node = Node(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

    for ai, di, i in zip(a, delta, range(1, P.T + 1)):
        node.update(ai, di, 1.0)
        z_bar[0, i] = node.x
        z_bar[1, i] = node.y
        z_bar[2, i] = node.v
        z_bar[3, i] = node.yaw

    return z_bar


def calc_linear_discrete_model(v, phi, delta):
    """
    calc linear and discrete time dynamic model.
    :param v: speed: v_bar
    :param phi: angle of vehicle: phi_bar
    :param delta: steering angle: delta_bar
    :return: A, B, C
    """

    A = np.array([[1.0, 0.0, P.dt * math.cos(phi), - P.dt * v * math.sin(phi)],
                  [0.0, 1.0, P.dt * math.sin(phi), P.dt * v * math.cos(phi)],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, P.dt * math.tan(delta) / P.WB, 1.0]])

    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [P.dt, 0.0],
                  [0.0, P.dt * v / (P.WB * math.cos(delta) ** 2)]])

    C = np.array([P.dt * v * math.sin(phi) * phi,
                  -P.dt * v * math.cos(phi) * phi,
                  0.0,
                  -P.dt * v * delta / (P.WB * math.cos(delta) ** 2)])

    return A, B, C


def solve_linear_mpc(z_ref, z_bar, z0, d_bar):
    """
    Solve the quadratic optimization problem using cvxpy and OSQP.
    No obstacle avoidance penalty needed when using artificial reference path.
    :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
    :param z_bar: predicted states in T steps
    :param z0: initial state
    :param d_bar: previous steering sequence
    :return: optimal acceleration and steering sequence, predicted trajectory
    """
    z = cvxpy.Variable((P.NX, P.T + 1))
    u = cvxpy.Variable((P.NU, P.T))

    cost = 0.0
    constrains = []

    for t in range(P.T):
        cost += cvxpy.quad_form(u[:, t], P.R)
        cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], P.Q)
        A, B, C = calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])
        constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]
        if t < P.T - 1:
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], P.Rd)
            constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= P.steer_change_max * P.dt]
    cost += cvxpy.quad_form(z_ref[:, P.T] - z[:, P.T], P.Qf)

    constrains += [z[:, 0] == z0]
    constrains += [z[2, :] <= P.speed_max]
    constrains += [z[2, :] >= P.speed_min]
    constrains += [cvxpy.abs(u[0, :]) <= P.acceleration_max]
    constrains += [cvxpy.abs(u[1, :]) <= P.steer_max]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    try:
        prob.solve(solver=cvxpy.OSQP)
    except Exception as e:
        print(f"OSQP solve failed: {e}")
        return [0.0] * P.T, [0.0] * P.T, None, None, None, None

    a, delta, x, y, yaw, v = None, None, None, None, None, None

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = z.value[0, :]
        y = z.value[1, :]
        v = z.value[2, :]
        yaw = z.value[3, :]
        a = u.value[0, :]
        delta = u.value[1, :]
    else:
        print("Cannot solve linear mpc!")
        a = [0.0] * P.T
        delta = [0.0] * P.T
    return a, delta, x, y, yaw, v

def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    design appropriate speed strategy
    :param cx: x of reference path [m]
    :param cy: y of reference path [m]
    :param cyaw: yaw of reference path [m]
    :param target_speed: target speed [m/s]
    :return: speed profile
    """

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi

    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle


def main():
    # 1. 非闭环任务（直线、曲线）
    tasks = [
        {
            "name": "Straight Path",
            "ax": [0.0, 20.0, 40.0, 60.0, 80.0],
            "ay": [0.0, 0.0, 0.0, 0.0, 0.0],
            "obstacle": None
        },
        {
            "name": "Curve Path",
            "ax": [0.0, 15.0, 30.0, 50.0, 60.0],
            "ay": [0.0, 40.0, 15.0, 30.0, 0.0],
            "obstacle": None
        }
    ]

    # 2. 圆形轨迹任务（终点前平稳停下）
    R = 20.0
    N = 200
    theta = np.linspace(0, 2*np.pi, N, endpoint=False)
    cx_circle = R * np.cos(theta)
    cy_circle = R * np.sin(theta)
    cyaw_circle = theta + np.pi/2
    ck_circle = [1.0/R]*N

    # 指定终点索引（圆周3/4处）
    circle_goal_idx = int(N * 0.75)
    sp_circle = [P.target_speed] * N
    N_drop = min(30, N)  # 用30个点慢慢减速
    for i in range(N_drop):
        sp_circle[(circle_goal_idx - i) % N] = P.target_speed * (N_drop - i) / N_drop
    sp_circle[circle_goal_idx] = 0.0

    tasks.append({
        "name": "Circle Path (Analytic)",
        "ax": cx_circle.tolist(),
        "ay": cy_circle.tolist(),
        "cyaw": cyaw_circle.tolist(),
        "ck": ck_circle,
        "analytic_circle": True,
        "sp": sp_circle,
        "goal_idx": circle_goal_idx
    })

    for task in tasks:
        print(f"\n======== Running Task: {task['name']} ========")
        if task.get("analytic_circle"):
            cx = task["ax"]
            cy = task["ay"]
            cyaw = task["cyaw"]
            ck = task["ck"]
            sp = task["sp"]
            circle_goal_idx = task["goal_idx"]
        else:
            cx, cy, cyaw, ck, _ = cs.calc_spline_course(
                task["ax"], task["ay"], ds=P.d_dist
            )
            sp = calc_speed_profile(cx, cy, cyaw, P.target_speed)

        ref_path = PATH(cx, cy, cyaw, ck)
        node = Node(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
        a_old, delta_old = None, None
        time = 0.0
        step_count = 0
        max_steps = 5000

        x = [node.x]
        y = [node.y]

        # ========== 性能统计变量 ==========
        tracking_errors = []
        ctrl_a = []
        ctrl_delta = []
        step_times = []
        v_log = []

        plt.figure(figsize=(10, 6))
        while True:
            import time as tmod
            t0 = tmod.time()
            # 生成z_ref
            z_ref = np.zeros((P.NX, P.T+1))
            if task.get("analytic_circle"):
                ind, _ = ref_path.nearest_index(node)
                Np = len(cx)
                for i in range(P.T+1):
                    idx = (ind + i) % Np
                    z_ref[0,i] = cx[idx]
                    z_ref[1,i] = cy[idx]
                    z_ref[2,i] = sp[idx]      # 严格使用自定义sp
                    z_ref[3,i] = cyaw[idx]
            else:
                z_ref, _ = calc_ref_trajectory_in_T_step(node, ref_path, sp)

            z0 = [node.x, node.y, node.v, node.yaw]
            a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = \
                linear_mpc_control(z_ref, z0, a_old, delta_old)

            a_cmd = a_opt[0] if a_opt is not None else 0.0
            d_cmd = delta_opt[0] if delta_opt is not None else 0.0
            node.update(a_cmd, d_cmd, 1.0)

            x.append(node.x)
            y.append(node.y)
            a_old, delta_old = a_opt, delta_opt
            step_count += 1

            # ====== 性能统计 ======
            tgt_idx, _ = ref_path.nearest_index(node)
            error = math.hypot(node.x - cx[tgt_idx], node.y - cy[tgt_idx])
            tracking_errors.append(error)
            ctrl_a.append(a_cmd)
            ctrl_delta.append(d_cmd)
            step_times.append(tmod.time() - t0)
            v_log.append(node.v)

            # 终止条件
            if task.get("analytic_circle"):
                dist_goal = math.hypot(node.x - cx[circle_goal_idx], node.y - cy[circle_goal_idx])
                if dist_goal < P.dist_stop:
                    print("Circle Path: Reached the goal point, simulation finished!")
                    break
                if step_count > max_steps:
                    print("Circle path finished (max steps reached).")
                    break
            else:
                dist = math.hypot(node.x - cx[-1], node.y - cy[-1])
                if dist < P.dist_stop and abs(node.v) < P.speed_stop:
                    print("Reached the goal point, simulation finished!")
                    break
                if step_count > 2000:
                    print("Step overflow, force break")
                    break

            # 画图
            plt.cla()
            if task.get("analytic_circle"):
                plt.plot(np.append(cx, cx[0]), np.append(cy, cy[0]), color='gray', label='Reference Path')
                plt.plot(cx[circle_goal_idx], cy[circle_goal_idx], 'mo', markersize=8, label='Circle End')
            else:
                plt.plot(cx, cy, color='gray', label='Reference Path')

            if x_opt is not None:
                plt.plot(x_opt, y_opt, color='darkviolet', marker='*', label='MPC Predicted')
            plt.plot(x, y, '-b', label='Actual Path')
            plt.plot(cx[tgt_idx], cy[tgt_idx], 'ro', label='Target Point')
            draw.draw_car(node.x, node.y, node.yaw, d_cmd, P)
            plt.axis("equal")
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys(), loc='best')
            plt.pause(0.001)
        plt.show()

        # ======= 仿真结束后：性能指标输出和作图 =======

        tracking_errors = np.array(tracking_errors)
        ctrl_a = np.array(ctrl_a)
        ctrl_delta = np.array(ctrl_delta)
        step_times = np.array(step_times)
        v_log = np.array(v_log)
        if len(tracking_errors) > 0:
            rmse = np.sqrt(np.mean(tracking_errors**2))
            max_err = np.max(np.abs(tracking_errors))
            mae = np.mean(np.abs(tracking_errors))
            mean_a = np.mean(np.abs(np.diff(ctrl_a)))
            mean_delta = np.mean(np.abs(np.diff(ctrl_delta)))
            mean_step_time = np.mean(step_times) * 1000  # ms
            print(f"\n==== Performance Metrics for {task['name']} ====")
            print(f"RMSE: {rmse:.3f} m")
            print(f"Max Error: {max_err:.3f} m")
            print(f"MAE: {mae:.3f} m")
            print(f"Mean |Δa|: {mean_a:.3f} m/s²")
            print(f"Mean |Δδ|: {mean_delta:.3f} rad")
            print(f"Avg Step Time: {mean_step_time:.2f} ms")

            # 绘制误差和控制输入等曲线

            plt.figure(figsize=(10,6))
            plt.subplot(4,1,1)
            plt.plot(tracking_errors)
            plt.ylabel('Tracking Error [m]')
            plt.title(f"{task['name']} Tracking Performance")
            plt.subplot(4,1,2)
            plt.plot(ctrl_a)
            plt.ylabel('Acc [m/s²]')
            plt.subplot(4,1,3)
            plt.plot(ctrl_delta)
            plt.ylabel('Steering [rad]')
            plt.subplot(4,1,4)
            plt.plot(v_log)
            plt.ylabel('Speed [m/s]')
            plt.xlabel('Step')
            plt.tight_layout()
            plt.show()



if __name__ == "__main__":
    main()
