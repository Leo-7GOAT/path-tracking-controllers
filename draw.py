import math

import matplotlib.pyplot as plt
import numpy as np


def _rotation_matrix(yaw):
    return np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
        dtype=float,
    )


def draw_car(x, y, yaw, steer, vehicle_config, color="tab:blue", ax=None):
    ax = ax or plt.gca()

    wheel_base = getattr(vehicle_config, "wheel_base", getattr(vehicle_config, "WB", 2.7))
    width = getattr(vehicle_config, "width", getattr(vehicle_config, "W", 1.85))
    front = getattr(vehicle_config, "front_overhang", getattr(vehicle_config, "RF", 3.6))
    rear = getattr(vehicle_config, "rear_overhang", getattr(vehicle_config, "RB", 1.0))
    tyre_width = getattr(vehicle_config, "tyre_width", getattr(vehicle_config, "TW", 0.22))
    tyre_radius = getattr(vehicle_config, "tyre_radius", getattr(vehicle_config, "TR", 0.33))

    outline = np.array(
        [
            [front, front, -rear, -rear, front],
            [width / 2.0, -width / 2.0, -width / 2.0, width / 2.0, width / 2.0],
        ]
    )

    wheel = np.array(
        [
            [tyre_radius, tyre_radius, -tyre_radius, -tyre_radius, tyre_radius],
            [-tyre_width, tyre_width, tyre_width, -tyre_width, -tyre_width],
        ]
    )
    wheel_offset = width * 0.35

    fr_wheel = wheel + np.array([[wheel_base], [-wheel_offset]])
    fl_wheel = wheel + np.array([[wheel_base], [wheel_offset]])
    rr_wheel = wheel + np.array([[0.0], [-wheel_offset]])
    rl_wheel = wheel + np.array([[0.0], [wheel_offset]])

    rot_body = _rotation_matrix(yaw)
    rot_steer = _rotation_matrix(yaw + steer)

    outline = rot_body.dot(outline) + np.array([[x], [y]])
    fr_wheel = rot_steer.dot(fr_wheel) + np.array([[x], [y]])
    fl_wheel = rot_steer.dot(fl_wheel) + np.array([[x], [y]])
    rr_wheel = rot_body.dot(rr_wheel) + np.array([[x], [y]])
    rl_wheel = rot_body.dot(rl_wheel) + np.array([[x], [y]])

    ax.plot(outline[0], outline[1], color=color, linewidth=1.2)
    ax.plot(fr_wheel[0], fr_wheel[1], color=color, linewidth=1.0)
    ax.plot(fl_wheel[0], fl_wheel[1], color=color, linewidth=1.0)
    ax.plot(rr_wheel[0], rr_wheel[1], color=color, linewidth=1.0)
    ax.plot(rl_wheel[0], rl_wheel[1], color=color, linewidth=1.0)
