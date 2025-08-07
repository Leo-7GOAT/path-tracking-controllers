"""
PID_Controller_Pure.py
======================

第四版：**完全去除曲率前馈，只保留纯 PID**（横向误差 e_f + 航向误差 θ_e）。

> 该版本用于与 Stanley / LQR / MPC 进行公平对比，也方便观察前馈的收益。

### 调参要点
| 项目 | 值 | 说明 |
|------|----|------|
| `KP_EF` | **1.8** | 提高对横向误差灵敏度，弥补去掉前馈后的不足 |
| `KI_EF` | **0.05** | 消除稳态误差，加入抗饱和 |
| `KD_EF` | **0.15** | 增大阻尼，压制超调 |
| `KP_THETA` | **2.2** | 加快航向对准 |

其它结构与前一版一致，包括积分抗饱和。
"""

import math
import os
import sys
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np

# 路径兼容
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")

import CurvesGenerator.cubic_spline as cs
import Control.draw as draw


class C:
    """Global parameters & tunables."""

    # ▶ Simulation
    dt = 0.1  # [s]
    T_max = 100.0
    d_stop = 0.5  # [m]

    # ▶ Vehicle geometry
    RF = 3.3
    RB = 0.8
    W = 2.4
    WD = 0.7 * W
    WB = 2.5
    MAX_STEER = 0.65
    TR = 0.44
    TW = 0.7

    # ▶ Longitudinal PID (speed)
    KP_V = 1.0
    KI_V = 0.1
    KD_V = 0.05

    # ▶ Lateral PID (pure PID on lateral error)
    KP_EF = 1.8
    KI_EF = 0.05
    KD_EF = 0.15
    KP_THETA = 2.2  # proportional on heading error

    TARGET_SPEED = 25.0 / 3.6  # [m/s] ≈ 25 km/h


class Node:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a: float, delta: float):
        delta = self._limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.v += a * C.dt

    @staticmethod
    def _limit_input(delta: float) -> float:
        return max(-C.MAX_STEER, min(C.MAX_STEER, delta))


# ────────────────────────────────────────────────────────────────────────────
# Helper functions

def pi_2_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Trajectory:
    """Stores reference path & provides error calculations."""

    def __init__(self, cx, cy, cyaw):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self._ind_prev = 0

    def calc_errors(self, node: Node) -> Tuple[float, float, int]:
        # front axle position
        fx = node.x + C.WB * math.cos(node.yaw)
        fy = node.y + C.WB * math.sin(node.yaw)
        dx = [fx - ix for ix in self.cx]
        dy = [fy - iy for iy in self.cy]

        ind = int(np.argmin(np.hypot(dx, dy)))
        ind = max(ind, self._ind_prev)
        self._ind_prev = ind

        vec = np.array([[dx[ind]], [dy[ind]]])
        front_norm = np.array([[math.cos(node.yaw - math.pi / 2.0)],
                               [math.sin(node.yaw - math.pi / 2.0)]])
        e_f = float(np.dot(vec.T, front_norm))
        theta_e = pi_2_pi(self.cyaw[ind] - node.yaw)
        return theta_e, e_f, ind


class PID:
    """Discrete PID with anti-windup."""

    def __init__(self, kp, ki, kd, i_max=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_err = 0.0
        self.i_max = i_max

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float) -> float:
        # Proportional
        p = self.kp * err

        # Integral with clamp
        self.integral += err * C.dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))
        i = self.ki * self.integral

        # Derivative
        d = self.kd * (err - self.prev_err) / C.dt
        self.prev_err = err

        return p + i + d


class ControllerPurePID:
    """Speed PID + Lateral pure PID (no feedforward)."""

    def __init__(self):
        self.pid_speed = PID(C.KP_V, C.KI_V, C.KD_V)
        self.pid_lat = PID(C.KP_EF, C.KI_EF, C.KD_EF)

    def control(self, node: Node, traj: Trajectory):
        # Longitudinal control
        a_cmd = self.pid_speed.step(C.TARGET_SPEED - node.v)

        # Lateral control
        theta_e, e_f, _ = traj.calc_errors(node)
        delta_pid = self.pid_lat.step(e_f)
        delta = delta_pid + C.KP_THETA * theta_e
        delta = Node._limit_input(delta)
        return a_cmd, delta, theta_e, e_f


# ────────────────────────────────────────────────────────────────────────────
# Reference path

def build_reference_path() -> Trajectory:
    # same wavy spline path as before
    ax = np.arange(0, 50, 0.5)
    ay = [math.sin(ix / 5.0) * ix / 2.0 for ix in ax]
    cx, cy, cyaw, *_ = cs.calc_spline_course(ax, ay, ds=C.dt)
    return Trajectory(cx, cy, cyaw)


# ────────────────────────────────────────────────────────────────────────────
# Simulation main loop

def main():
    traj = build_reference_path()
    node = Node(traj.cx[0], traj.cy[0], traj.cyaw[0], v=0.0)
    controller = ControllerPurePID()

    xs, ys = [], []
    time = 0.0
    plt.ion()

    while time < C.T_max:
        a, delta, theta_e, e_f = controller.control(node, traj)
        node.update(a, delta)
        time += C.dt

        xs.append(node.x)
        ys.append(node.y)

        if math.hypot(node.x - traj.cx[-1], node.y - traj.cy[-1]) <= C.d_stop:
            print("Reached goal.")
            break

        # Visualise
        plt.cla()
        plt.plot(traj.cx, traj.cy, color='gray', linewidth=2, label='reference')
        plt.plot(xs, ys, color='magenta', linewidth=2, label='trajectory')
        plt.plot(traj.cx[traj._ind_prev], traj.cy[traj._ind_prev], '.r')

        steer_vis = -math.atan(C.WB * delta / (node.v + 1e-5))
        draw.draw_car(node.x, node.y, node.yaw, steer_vis, C)

        plt.title(f"Pure PID | v={node.v*3.6:4.1f} km/h  e_f={e_f: .2f} m")
        plt.axis("equal")
        plt.legend()
        plt.pause(0.001)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
