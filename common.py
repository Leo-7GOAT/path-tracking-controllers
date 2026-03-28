import math
from dataclasses import dataclass, field
from typing import List

import numpy as np


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PID1D:
    def __init__(
        self,
        kp,
        ki,
        kd,
        integral_limit=5.0,
        integral_decay=1.0,
        sign_change_decay=1.0,
        error_deadband=0.0,
        derivative_smoothing=0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral_decay = clamp(integral_decay, 0.0, 1.0)
        self.sign_change_decay = clamp(sign_change_decay, 0.0, 1.0)
        self.error_deadband = max(0.0, error_deadband)
        self.derivative_smoothing = clamp(derivative_smoothing, 0.0, 0.95)
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def step(self, error, dt):
        if abs(error) < self.error_deadband:
            error = 0.0
        if error * self.prev_error < 0.0:
            self.integral *= self.sign_change_decay
        self.integral += error * dt
        self.integral *= self.integral_decay
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        raw_derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        derivative = (
            self.derivative_smoothing * self.prev_derivative
            + (1.0 - self.derivative_smoothing) * raw_derivative
        )
        self.prev_error = error
        self.prev_derivative = derivative
        return self.kp * error + self.ki * self.integral + self.kd * derivative


@dataclass
class VehicleConfig:
    wheel_base: float = 2.7
    max_steer: float = math.radians(35.0)
    max_accel: float = 2.5
    max_speed: float = 16.0
    min_speed: float = 0.0
    width: float = 1.85
    front_overhang: float = 3.6
    rear_overhang: float = 1.0
    tyre_width: float = 0.22
    tyre_radius: float = 0.33


@dataclass
class SimulationConfig:
    dt: float = 0.1
    max_time: float = 45.0
    target_speed: float = 8.0
    stop_distance: float = 1.0
    stop_speed: float = 0.5
    search_window: int = 80
    target_speed_slew_rate: float = 2.2
    comfortable_brake: float = 1.1


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    v: float = 0.0
    steer: float = 0.0

    def step(self, accel, steer, dt, vehicle_config):
        steer = clamp(steer, -vehicle_config.max_steer, vehicle_config.max_steer)
        accel = clamp(accel, -vehicle_config.max_accel, vehicle_config.max_accel)

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(
            self.yaw + self.v / vehicle_config.wheel_base * math.tan(steer) * dt
        )
        self.v = clamp(self.v + accel * dt, vehicle_config.min_speed, vehicle_config.max_speed)
        self.steer = steer


class ReferencePath:
    def __init__(self, cx, cy, cyaw, ck, s, search_window=80):
        self.cx = np.asarray(cx, dtype=float)
        self.cy = np.asarray(cy, dtype=float)
        self.cyaw = np.asarray(cyaw, dtype=float)
        self.ck = np.asarray(ck, dtype=float)
        self.s = np.asarray(s, dtype=float)
        self.last_index = 0
        self.search_window = int(search_window)

    @property
    def length(self):
        return int(len(self.cx))

    def reset(self):
        self.last_index = 0

    def goal_distance(self, state):
        return math.hypot(state.x - self.cx[-1], state.y - self.cy[-1])

    def nearest_index(self, x, y, start_index=None, search_window=None):
        index = self.peek_nearest_index(x, y, start_index=start_index, search_window=search_window)
        self.last_index = max(self.last_index, index)
        return index

    def peek_nearest_index(self, x, y, start_index=None, search_window=None):
        if self.length == 0:
            return 0

        start = self.last_index if start_index is None else int(start_index)
        start = clamp(start, 0, self.length - 1)
        window = search_window if search_window is not None else self.search_window
        end = min(self.length, start + max(1, int(window)))

        dx = self.cx[start:end] - x
        dy = self.cy[start:end] - y
        if dx.size == 0:
            return self.length - 1

        local_index = int(np.argmin(np.hypot(dx, dy)))
        return start + local_index

    def tracking_error(self, state, start_index=None):
        index = self.nearest_index(state.x, state.y, start_index=start_index)
        dx = state.x - self.cx[index]
        dy = state.y - self.cy[index]
        normal = np.array([-math.sin(self.cyaw[index]), math.cos(self.cyaw[index])])
        lateral_error = float(dx * normal[0] + dy * normal[1])
        heading_error = normalize_angle(state.yaw - self.cyaw[index])
        return index, lateral_error, heading_error, float(self.ck[index])

    def front_axle_error(self, state, vehicle_config, start_index=None):
        fx = state.x + vehicle_config.wheel_base * math.cos(state.yaw)
        fy = state.y + vehicle_config.wheel_base * math.sin(state.yaw)
        index = self.nearest_index(fx, fy, start_index=start_index)
        dx = fx - self.cx[index]
        dy = fy - self.cy[index]
        normal = np.array(
            [
                math.cos(state.yaw - math.pi / 2.0),
                math.sin(state.yaw - math.pi / 2.0),
            ]
        )
        lateral_error = float(dx * normal[0] + dy * normal[1])
        heading_error = normalize_angle(self.cyaw[index] - state.yaw)
        return index, lateral_error, heading_error, float(self.ck[index])


@dataclass
class Scenario:
    name: str
    path: ReferencePath
    speed_profile: np.ndarray
    goal_index: int = -1
    stop_distance: float = -1.0
    require_stop: bool = True


@dataclass
class SimulationRecord:
    time: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    yaw: List[float] = field(default_factory=list)
    speed: List[float] = field(default_factory=list)
    target_speed: List[float] = field(default_factory=list)
    accel: List[float] = field(default_factory=list)
    steer: List[float] = field(default_factory=list)
    lateral_error: List[float] = field(default_factory=list)
    heading_error: List[float] = field(default_factory=list)
    target_index: List[int] = field(default_factory=list)


@dataclass
class BenchmarkResult:
    scenario: str
    controller: str
    success: bool
    avg_lateral_error: float
    max_lateral_error: float
    rmse_lateral_error: float
    completion_time: float
    mean_abs_speed_error: float
    mean_abs_steer: float
    mean_abs_steer_rate: float

    def to_dict(self):
        return {
            "Scenario": self.scenario,
            "Controller": self.controller,
            "Success": "Yes" if self.success else "No",
            "Avg Lat Err (m)": "{:.3f}".format(self.avg_lateral_error),
            "Max Lat Err (m)": "{:.3f}".format(self.max_lateral_error),
            "RMSE (m)": "{:.3f}".format(self.rmse_lateral_error),
            "Completion Time (s)": "{:.2f}".format(self.completion_time),
            "Mean |Speed Err| (m/s)": "{:.3f}".format(self.mean_abs_speed_error),
            "Mean |Steer| (rad)": "{:.3f}".format(self.mean_abs_steer),
            "Mean |Steer Rate| (rad/s)": "{:.3f}".format(self.mean_abs_steer_rate),
        }


def format_table(results):
    rows = [result.to_dict() for result in results]
    if not rows:
        return ""

    headers = list(rows[0].keys())
    widths = []
    for header in headers:
        width = len(header)
        for row in rows:
            width = max(width, len(str(row[header])))
        widths.append(width)

    lines = []
    header_line = " | ".join(header.ljust(width) for header, width in zip(headers, widths))
    split_line = "-+-".join("-" * width for width in widths)
    lines.append(header_line)
    lines.append(split_line)

    for row in rows:
        lines.append(
            " | ".join(str(row[header]).ljust(width) for header, width in zip(headers, widths))
        )
    return "\n".join(lines)
