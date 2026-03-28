import argparse
import csv
import math
import os
import shutil
from contextlib import contextmanager, nullcontext

from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
import numpy as np

from common import (
    BenchmarkResult,
    ReferencePath,
    Scenario,
    SimulationConfig,
    SimulationRecord,
    VehicleConfig,
    VehicleState,
    clamp,
    format_table,
)
from cubic_spline import calc_spline_course
from draw import draw_car


COLORS = {
    "Stanley": "tab:red",
    "PID": "tab:blue",
    "PurePursuit": "tab:brown",
    "LQR": "tab:green",
    "MPC": "tab:purple",
}


def _slugify(name):
    return name.lower().replace(" ", "_")


def _prepare_display_backend(enable_display):
    if not enable_display:
        return

    backend = str(plt.get_backend()).lower()
    interactive_backends = ("tkagg", "qtagg", "qt5agg", "wxagg", "macosx")
    if any(name in backend for name in interactive_backends):
        return

    for candidate in ("TkAgg", "QtAgg"):
        try:
            plt.switch_backend(candidate)
            return
        except Exception:
            continue


def _switch_to_headless_backend():
    plt.close("all")
    backend = str(plt.get_backend()).lower()
    if "agg" in backend:
        return
    try:
        plt.switch_backend("Agg")
    except Exception:
        pass


def _reset_output_dir(output_dir):
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)


@contextmanager
def _suppress_interactive_windows():
    was_interactive = plt.isinteractive()
    plt.ioff()
    try:
        yield
    finally:
        if was_interactive:
            plt.ion()


def _goal_index(scenario):
    return scenario.goal_index if scenario.goal_index >= 0 else scenario.path.length - 1


def _stop_speed_limit(distance_to_goal, stop_distance, comfortable_brake, stop_buffer_scale=0.55):
    usable_distance = max(distance_to_goal - stop_distance * stop_buffer_scale, 0.0)
    return math.sqrt(max(0.0, 2.0 * comfortable_brake * usable_distance))


def build_speed_profile(length, cruise_speed, decel_points=35):
    profile = np.full(length, cruise_speed, dtype=float)
    decel_points = min(decel_points, max(1, length))
    start = max(0, length - decel_points)
    profile[start:] = np.linspace(cruise_speed, 0.0, length - start)
    profile[-1] = 0.0
    return profile


def build_constant_speed_profile(length, cruise_speed):
    return np.full(length, cruise_speed, dtype=float)


def build_spline_scenario(name, ax, ay, cruise_speed, ds=0.25, search_window=80, stop_distance=1.35):
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=ds)
    return Scenario(
        name=name,
        path=ReferencePath(cx, cy, cyaw, ck, s, search_window=search_window),
        speed_profile=build_speed_profile(len(cx), cruise_speed),
        goal_index=len(cx) - 1,
        stop_distance=stop_distance,
        require_stop=True,
    )


def build_circle_arc_scenario(radius, cruise_speed, search_window=80):
    theta = np.linspace(0.0, 2.0 * math.pi, 320, endpoint=False)
    cx = radius * np.cos(theta)
    cy = radius * np.sin(theta)
    cyaw = theta + math.pi / 2.0
    ck = np.full_like(theta, 1.0 / radius)
    s = radius * theta
    return Scenario(
        name="CircleLoop",
        path=ReferencePath(cx, cy, cyaw, ck, s, search_window=search_window),
        speed_profile=build_constant_speed_profile(len(cx), cruise_speed),
        goal_index=len(cx) - 1,
        stop_distance=1.8,
        require_stop=False,
    )


class LiveRunViewer:
    def __init__(self, vehicle_config, pause_time=0.001, redraw_stride=2):
        self.vehicle_config = vehicle_config
        self.pause_time = pause_time
        self.redraw_stride = max(1, int(redraw_stride))
        self.figure = None
        self.axis = None
        self._last_title = None

    def _ensure_figure(self):
        if self.figure is None or self.axis is None or not plt.fignum_exists(self.figure.number):
            plt.ion()
            self.figure, self.axis = plt.subplots(figsize=(9, 7))
            manager = getattr(self.figure.canvas, "manager", None)
            if manager is not None and hasattr(manager, "set_window_title"):
                manager.set_window_title("Path Tracking Live Demo")
            plt.show(block=False)

    def update(self, scenario, controller_name, record, step_index):
        if not record.x or step_index % self.redraw_stride != 0:
            return

        self._ensure_figure()
        color = COLORS.get(controller_name, "tab:blue")
        idx = len(record.x) - 1
        goal_index = _goal_index(scenario)

        self.axis.clear()
        self.axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.0, label="Reference")
        self.axis.plot(record.x, record.y, color=color, linewidth=2.0, label=controller_name)
        self.axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=35, label="Start")
        self.axis.scatter(
            [scenario.path.cx[goal_index]],
            [scenario.path.cy[goal_index]],
            color="tab:pink",
            s=35,
            label="Goal",
        )
        draw_car(
            record.x[idx],
            record.y[idx],
            record.yaw[idx],
            record.steer[idx],
            self.vehicle_config,
            color=color,
            ax=self.axis,
        )
        self.axis.set_xlabel("x [m]")
        self.axis.set_ylabel("y [m]")
        self.axis.axis("equal")
        self.axis.grid(True, alpha=0.3)
        self.axis.legend(frameon=False, loc="best")
        self.axis.set_title(
            "{} | {} | t={:.1f}s | v={:.2f}m/s".format(
                controller_name,
                scenario.name,
                record.time[idx],
                record.speed[idx],
            )
        )
        self.axis.text(
            0.02,
            0.02,
            "lat={:.3f} m | steer={:.3f} rad | target={:.2f} m/s".format(
                abs(record.lateral_error[idx]),
                record.steer[idx],
                record.target_speed[idx],
            ),
            transform=self.axis.transAxes,
            fontsize=10,
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
        )
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(self.pause_time)

    def finish_run(self, scenario, controller_name, result):
        if self.figure is None or self.axis is None or not plt.fignum_exists(self.figure.number):
            return
        self.axis.set_title(
            "{} | {} | success={} | RMSE={:.3f} m".format(
                controller_name,
                scenario.name,
                "Yes" if result.success else "No",
                result.rmse_lateral_error,
            )
        )
        self.figure.canvas.draw_idle()
        plt.pause(0.6)

    def close(self):
        if self.figure is None:
            return
        plt.ioff()
        if plt.fignum_exists(self.figure.number):
            plt.show()
            plt.close(self.figure)
        self.figure = None
        self.axis = None


def play_multi_controller_live(scenarios, history, controller_names, vehicle_config, pause_time=0.03, frame_stride=2):
    plt.ion()
    figure, axis = plt.subplots(figsize=(9, 7))
    manager = getattr(figure.canvas, "manager", None)
    if manager is not None and hasattr(manager, "set_window_title"):
        manager.set_window_title("All Controllers Live Demo")

    for scenario in scenarios:
        goal_index = _goal_index(scenario)
        max_len = max(len(history[(scenario.name, controller_name)].x) for controller_name in controller_names)
        stride = max(1, int(frame_stride))
        frame_indices = list(range(0, max_len, stride))
        if max_len > 0 and frame_indices[-1] != max_len - 1:
            frame_indices.append(max_len - 1)

        for frame_idx in frame_indices:
            axis.clear()
            axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.2, label="Reference")
            axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=35, label="Start")
            axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=35, label="Goal")

            time_label = 0.0
            for controller_name in controller_names:
                record = history[(scenario.name, controller_name)]
                color = COLORS.get(controller_name, "tab:blue")
                idx = min(frame_idx, len(record.x) - 1)
                if idx < 0:
                    continue
                time_label = max(time_label, record.time[idx])
                axis.plot(record.x[: idx + 1], record.y[: idx + 1], color=color, linewidth=1.9, label=controller_name)
                draw_car(
                    record.x[idx],
                    record.y[idx],
                    record.yaw[idx],
                    record.steer[idx],
                    vehicle_config,
                    color=color,
                    ax=axis,
                )

            axis.set_title("{} | All Controllers | t={:.1f}s".format(scenario.name, time_label))
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.axis("equal")
            axis.grid(True, alpha=0.3)
            axis.legend(frameon=False, ncol=min(4, len(controller_names) + 1), loc="best")
            figure.tight_layout()
            figure.canvas.draw_idle()
            figure.canvas.flush_events()
            plt.pause(pause_time)

        plt.pause(0.5)

    plt.ioff()
    if plt.fignum_exists(figure.number):
        plt.show()
        plt.close(figure)


def build_scenarios(sim_config):
    return [
        build_spline_scenario(
            "Straight",
            ax=np.arange(0.0, 70.0, 6.0),
            ay=np.zeros(12),
            cruise_speed=sim_config.target_speed,
            search_window=sim_config.search_window,
            stop_distance=1.35,
        ),
        build_spline_scenario(
            "Curved",
            ax=[0.0, 8.0, 18.0, 30.0, 42.0, 55.0, 68.0, 82.0],
            ay=[0.0, 2.5, 16.0, -12.0, -19.0, 20.0, 13.0, 0.0],
            cruise_speed=sim_config.target_speed * 0.88,
            search_window=sim_config.search_window,
            stop_distance=1.7,
        ),
        build_circle_arc_scenario(
            radius=18.0,
            cruise_speed=sim_config.target_speed * 0.65,
            search_window=sim_config.search_window,
        ),
    ]


def run_benchmark(controller_name, controller_cls, scenario, sim_config, vehicle_config, live_viewer=None):
    scenario.path.reset()
    controller = controller_cls(dt=sim_config.dt, vehicle_config=vehicle_config)
    goal_index = _goal_index(scenario)
    goal_x = float(scenario.path.cx[goal_index])
    goal_y = float(scenario.path.cy[goal_index])
    stop_distance = scenario.stop_distance if scenario.stop_distance > 0.0 else sim_config.stop_distance
    require_stop = scenario.require_stop
    goal_index_tolerance = 5
    state = VehicleState(
        x=float(scenario.path.cx[0]),
        y=float(scenario.path.cy[0]),
        yaw=float(scenario.path.cyaw[0]),
        v=0.0,
    )
    record = SimulationRecord()

    max_steps = int(sim_config.max_time / sim_config.dt)
    progress_index = 0
    success = False
    completion_time = sim_config.max_time
    target_speed_cmd = 0.0

    for step in range(max_steps):
        speed_index = scenario.path.peek_nearest_index(
            state.x,
            state.y,
            start_index=max(0, progress_index - 5),
            search_window=sim_config.search_window,
        )
        progress_index = max(progress_index, speed_index)
        remaining_distance = max(
            0.0,
            float(scenario.path.s[goal_index] - scenario.path.s[min(progress_index, goal_index)]),
        )
        preview_goal_distance = math.hypot(state.x - goal_x, state.y - goal_y)
        if require_stop:
            distance_limited_speed = min(
                _stop_speed_limit(remaining_distance, stop_distance, sim_config.comfortable_brake),
                _stop_speed_limit(preview_goal_distance, stop_distance, sim_config.comfortable_brake),
            )
            raw_target_speed = min(float(scenario.speed_profile[speed_index]), distance_limited_speed)
            if preview_goal_distance <= stop_distance * 1.15 and state.v <= max(sim_config.stop_speed * 1.5, 0.45):
                raw_target_speed = 0.0
        else:
            raw_target_speed = float(scenario.speed_profile[speed_index])

        max_speed_delta = sim_config.target_speed_slew_rate * sim_config.dt
        if raw_target_speed > target_speed_cmd:
            target_speed_cmd = min(raw_target_speed, target_speed_cmd + max_speed_delta)
        else:
            target_speed_cmd = max(raw_target_speed, target_speed_cmd - max_speed_delta)

        target_speed = max(0.0, target_speed_cmd)

        accel, steer, _ = controller.control(state, scenario.path, target_speed)
        if require_stop:
            terminal_zone = max(5.0, 3.8 * stop_distance)
            if preview_goal_distance <= terminal_zone:
                terminal_target_speed = min(
                    target_speed,
                    _stop_speed_limit(preview_goal_distance, stop_distance, sim_config.comfortable_brake),
                )
                terminal_accel = 1.15 * (terminal_target_speed - state.v) - 0.35 * state.v
                if preview_goal_distance <= stop_distance * 1.25:
                    terminal_accel -= 0.2 * state.v
                blend = max(0.0, min(1.0, (terminal_zone - preview_goal_distance) / terminal_zone))
                accel = (1.0 - blend) * accel + blend * terminal_accel
                accel = clamp(accel, -vehicle_config.max_accel, vehicle_config.max_accel)

        state.step(accel, steer, sim_config.dt, vehicle_config)

        index, lateral_error, heading_error, _ = scenario.path.tracking_error(
            state,
            start_index=max(0, scenario.path.last_index - 5),
        )

        current_time = (step + 1) * sim_config.dt
        record.time.append(current_time)
        record.x.append(state.x)
        record.y.append(state.y)
        record.yaw.append(state.yaw)
        record.speed.append(state.v)
        record.target_speed.append(target_speed)
        record.accel.append(accel)
        record.steer.append(steer)
        record.lateral_error.append(lateral_error)
        record.heading_error.append(heading_error)
        record.target_index.append(index)
        progress_index = max(progress_index, index)
        if live_viewer is not None:
            live_viewer.update(scenario, controller_name, record, step)

        goal_distance = math.hypot(state.x - goal_x, state.y - goal_y)
        if require_stop:
            at_goal = progress_index >= goal_index - goal_index_tolerance and goal_distance <= stop_distance * 1.05
            low_speed = abs(state.v) <= max(sim_config.stop_speed, 0.25)
        else:
            at_goal = progress_index >= goal_index - 1
            low_speed = True
        if at_goal and low_speed:
            success = True
            completion_time = current_time
            break

    lateral_error = np.abs(np.asarray(record.lateral_error))
    speed_error = np.abs(np.asarray(record.target_speed) - np.asarray(record.speed))
    steer = np.abs(np.asarray(record.steer))
    steer_rate = (
        np.abs(np.diff(np.asarray(record.steer)) / sim_config.dt)
        if len(record.steer) > 1
        else np.asarray([0.0])
    )

    result = BenchmarkResult(
        scenario=scenario.name,
        controller=controller_name,
        success=success,
        avg_lateral_error=float(np.mean(lateral_error)) if lateral_error.size else 0.0,
        max_lateral_error=float(np.max(lateral_error)) if lateral_error.size else 0.0,
        rmse_lateral_error=float(np.sqrt(np.mean(lateral_error ** 2))) if lateral_error.size else 0.0,
        completion_time=completion_time,
        mean_abs_speed_error=float(np.mean(speed_error)) if speed_error.size else 0.0,
        mean_abs_steer=float(np.mean(steer)) if steer.size else 0.0,
        mean_abs_steer_rate=float(np.mean(steer_rate)) if steer_rate.size else 0.0,
    )
    if live_viewer is not None:
        live_viewer.finish_run(scenario, controller_name, result)
    return result, record


def save_csv(results, output_path):
    rows = [result.to_dict() for result in results]
    with open(output_path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_markdown_table(results, output_path):
    rows = [result.to_dict() for result in results]
    headers = list(rows[0].keys())
    with open(output_path, "w", encoding="utf-8") as file:
        file.write("| " + " | ".join(headers) + " |\n")
        file.write("| " + " | ".join(["---"] * len(headers)) + " |\n")
        for row in rows:
            file.write("| " + " | ".join(str(row[header]) for header in headers) + " |\n")


def plot_trajectory_overview(scenarios, history, controller_names, output_path, vehicle_config):
    with _suppress_interactive_windows():
        figure, axes = plt.subplots(1, len(scenarios), figsize=(18, 5))
        if len(scenarios) == 1:
            axes = [axes]

        for axis, scenario in zip(axes, scenarios):
            goal_index = _goal_index(scenario)
            axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.0, label="Reference")
            for controller_name in controller_names:
                record = history[(scenario.name, controller_name)]
                axis.plot(
                    record.x,
                    record.y,
                    color=COLORS.get(controller_name, "tab:blue"),
                    linewidth=1.8,
                    label=controller_name,
                )

            axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=30, label="Start")
            axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=30, label="Goal")

            reference_controller = controller_names[-1]
            record = history[(scenario.name, reference_controller)]
            if record.x:
                draw_car(
                    record.x[-1],
                    record.y[-1],
                    record.yaw[-1],
                    record.steer[-1],
                    vehicle_config,
                    color=COLORS.get(reference_controller, "tab:blue"),
                    ax=axis,
                )

            axis.set_title("{} Path".format(scenario.name))
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.axis("equal")
            axis.grid(True, alpha=0.3)

        handles, labels = axes[0].get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        figure.legend(unique.values(), unique.keys(), loc="lower center", ncol=max(4, len(unique) // 2), frameon=False)
        figure.tight_layout(rect=[0, 0.08, 1, 1])
        figure.savefig(output_path, dpi=220)
        plt.close(figure)


def plot_metric_dashboard(results, output_path):
    scenarios = []
    controllers = []
    for result in results:
        if result.scenario not in scenarios:
            scenarios.append(result.scenario)
        if result.controller not in controllers:
            controllers.append(result.controller)

    with _suppress_interactive_windows():
        x_axis = np.arange(len(scenarios))
        width = 0.82 / max(1, len(controllers))
        figure, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

        for i, controller in enumerate(controllers):
            offsets = x_axis + (i - (len(controllers) - 1) / 2.0) * width
            rmse_values = []
            time_values = []
            for scenario in scenarios:
                matched = [result for result in results if result.scenario == scenario and result.controller == controller][0]
                rmse_values.append(matched.rmse_lateral_error)
                time_values.append(matched.completion_time)

            axes[0].bar(offsets, rmse_values, width=width, color=COLORS.get(controller, "tab:blue"), label=controller)
            axes[1].bar(offsets, time_values, width=width, color=COLORS.get(controller, "tab:blue"), label=controller)

        axes[0].set_ylabel("RMSE lateral error [m]")
        axes[1].set_ylabel("Completion time [s]")
        axes[1].set_xticks(x_axis)
        axes[1].set_xticklabels(scenarios)
        axes[0].grid(True, axis="y", alpha=0.3)
        axes[1].grid(True, axis="y", alpha=0.3)
        axes[0].legend(frameon=False, ncol=min(5, len(controllers)))
        figure.tight_layout()
        figure.savefig(output_path, dpi=220)
        plt.close(figure)


def plot_single_run(scenario, controller_name, result, record, output_path, vehicle_config, show=False):
    color = COLORS.get(controller_name, "tab:blue")
    context = _suppress_interactive_windows() if not show else nullcontext()
    with context:
        figure, axes = plt.subplots(2, 2, figsize=(13, 9))
        time_axis = np.asarray(record.time)

        axes[0, 0].plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.0, label="Reference")
        axes[0, 0].plot(record.x, record.y, color=color, linewidth=2.0, label=controller_name)
        axes[0, 0].scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=30, label="Start")
        goal_index = _goal_index(scenario)
        axes[0, 0].scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=30, label="Goal")
        if record.x:
            draw_car(
                record.x[-1],
                record.y[-1],
                record.yaw[-1],
                record.steer[-1],
                vehicle_config,
                color=color,
                ax=axes[0, 0],
            )
        axes[0, 0].set_title("Trajectory")
        axes[0, 0].set_xlabel("x [m]")
        axes[0, 0].set_ylabel("y [m]")
        axes[0, 0].axis("equal")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend(frameon=False)

        axes[0, 1].plot(time_axis, record.speed, color=color, linewidth=2.0, label="Speed")
        axes[0, 1].plot(time_axis, record.target_speed, color="black", linestyle="--", linewidth=1.5, label="Target Speed")
        axes[0, 1].set_title("Speed Tracking")
        axes[0, 1].set_xlabel("time [s]")
        axes[0, 1].set_ylabel("speed [m/s]")
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend(frameon=False)

        axes[1, 0].plot(time_axis, np.abs(record.lateral_error), color=color, linewidth=2.0, label="|Lateral Error|")
        axes[1, 0].plot(time_axis, np.abs(record.heading_error), color="tab:gray", linewidth=1.5, label="|Heading Error|")
        axes[1, 0].set_title("Tracking Error")
        axes[1, 0].set_xlabel("time [s]")
        axes[1, 0].set_ylabel("error")
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend(frameon=False)

        axes[1, 1].plot(time_axis, record.steer, color=color, linewidth=2.0, label="Steer")
        axes[1, 1].plot(time_axis, record.accel, color="tab:orange", linewidth=1.5, label="Accel")
        axes[1, 1].set_title("Control Input")
        axes[1, 1].set_xlabel("time [s]")
        axes[1, 1].set_ylabel("command")
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend(frameon=False)

        figure.suptitle(
            "{} | {} | success={} | RMSE={:.3f} m".format(
                controller_name,
                scenario.name,
                "Yes" if result.success else "No",
                result.rmse_lateral_error,
            )
        )
        figure.tight_layout(rect=[0, 0, 1, 0.96])
        figure.savefig(output_path, dpi=220)

        if show:
            return figure

        plt.close(figure)
        return None


def plot_controller_overview(scenarios, controller_name, history, output_path, vehicle_config):
    with _suppress_interactive_windows():
        figure, axes = plt.subplots(1, len(scenarios), figsize=(18, 5))
        if len(scenarios) == 1:
            axes = [axes]

        for axis, scenario in zip(axes, scenarios):
            record = history[(scenario.name, controller_name)]
            goal_index = _goal_index(scenario)
            axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.0, label="Reference")
            axis.plot(record.x, record.y, color=COLORS.get(controller_name, "tab:blue"), linewidth=2.0, label=controller_name)
            axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=30, label="Start")
            axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=30, label="Goal")
            if record.x:
                draw_car(
                    record.x[-1],
                    record.y[-1],
                    record.yaw[-1],
                    record.steer[-1],
                    vehicle_config,
                    color=COLORS.get(controller_name, "tab:blue"),
                    ax=axis,
                )
            axis.set_title("{} | {}".format(controller_name, scenario.name))
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.axis("equal")
            axis.grid(True, alpha=0.3)
            axis.legend(frameon=False)

        figure.tight_layout()
        figure.savefig(output_path, dpi=220)
        plt.close(figure)


def plot_multi_controller_snapshot(scenario, controller_names, history, output_path, vehicle_config):
    with _suppress_interactive_windows():
        figure, axis = plt.subplots(figsize=(8.5, 6.5))
        goal_index = _goal_index(scenario)

        axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.2, label="Reference")
        axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=35, label="Start")
        axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=35, label="Goal")

        for controller_name in controller_names:
            record = history[(scenario.name, controller_name)]
            color = COLORS.get(controller_name, "tab:blue")
            axis.plot(record.x, record.y, color=color, linewidth=1.9, alpha=0.9, label=controller_name)
            if record.x:
                draw_car(
                    record.x[-1],
                    record.y[-1],
                    record.yaw[-1],
                    record.steer[-1],
                    vehicle_config,
                    color=color,
                    ax=axis,
                )

        axis.set_title("{} | All Controllers".format(scenario.name))
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.axis("equal")
        axis.grid(True, alpha=0.3)
        axis.legend(frameon=False, ncol=min(4, len(controller_names) + 1))
        figure.tight_layout()
        figure.savefig(output_path, dpi=220)
        plt.close(figure)


def animate_single_run(
    scenario,
    controller_name,
    result,
    record,
    output_path=None,
    vehicle_config=None,
    show=False,
    frame_stride=2,
):
    color = COLORS.get(controller_name, "tab:blue")
    vehicle_config = vehicle_config or VehicleConfig()
    context = _suppress_interactive_windows() if not show else nullcontext()
    with context:
        frame_indices = list(range(0, len(record.x), max(1, int(frame_stride))))
        if record.x and frame_indices[-1] != len(record.x) - 1:
            frame_indices.append(len(record.x) - 1)

        figure, axis = plt.subplots(figsize=(8, 6))

        def _draw_frame(frame_no):
            idx = frame_indices[frame_no]
            goal_index = _goal_index(scenario)
            axis.clear()
            axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.0, label="Reference")
            axis.plot(record.x[: idx + 1], record.y[: idx + 1], color=color, linewidth=2.0, label=controller_name)
            axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=35, label="Start")
            axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=35, label="Goal")
            draw_car(
                record.x[idx],
                record.y[idx],
                record.yaw[idx],
                record.steer[idx],
                vehicle_config,
                color=color,
                ax=axis,
            )
            axis.set_title(
                "{} | {} | t={:.1f}s | v={:.2f}m/s".format(
                    controller_name,
                    scenario.name,
                    record.time[idx],
                    record.speed[idx],
                )
            )
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.axis("equal")
            axis.grid(True, alpha=0.3)
            axis.legend(frameon=False, loc="best")
            axis.text(
                0.02,
                0.02,
                "success={} | RMSE={:.3f} m".format("Yes" if result.success else "No", result.rmse_lateral_error),
                transform=axis.transAxes,
                fontsize=10,
                bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
            )

        animation = FuncAnimation(
            figure,
            _draw_frame,
            frames=len(frame_indices),
            interval=60,
            repeat=False,
        )

        if output_path:
            animation.save(output_path, writer=PillowWriter(fps=15))

        if show:
            return figure, animation

        plt.close(figure)
        return None, None


def animate_multi_controller_run(
    scenario,
    controller_names,
    history,
    output_path=None,
    vehicle_config=None,
    show=False,
    frame_stride=2,
):
    vehicle_config = vehicle_config or VehicleConfig()
    context = _suppress_interactive_windows() if not show else nullcontext()
    with context:
        figure, axis = plt.subplots(figsize=(8.5, 6.5))
        goal_index = _goal_index(scenario)
        stride = max(1, int(frame_stride))
        max_len = max(len(history[(scenario.name, controller_name)].x) for controller_name in controller_names)
        frame_indices = list(range(0, max_len, stride))
        if max_len > 0 and frame_indices[-1] != max_len - 1:
            frame_indices.append(max_len - 1)

        def _draw_frame(frame_no):
            axis.clear()
            frame_idx = frame_indices[frame_no]
            axis.plot(scenario.path.cx, scenario.path.cy, color="black", linewidth=2.2, label="Reference")
            axis.scatter([scenario.path.cx[0]], [scenario.path.cy[0]], color="tab:orange", s=35, label="Start")
            axis.scatter([scenario.path.cx[goal_index]], [scenario.path.cy[goal_index]], color="tab:pink", s=35, label="Goal")

            time_label = 0.0
            for controller_name in controller_names:
                record = history[(scenario.name, controller_name)]
                color = COLORS.get(controller_name, "tab:blue")
                idx = min(frame_idx, len(record.x) - 1)
                if idx < 0:
                    continue
                time_label = max(time_label, record.time[idx])
                axis.plot(record.x[: idx + 1], record.y[: idx + 1], color=color, linewidth=1.8, label=controller_name)
                draw_car(
                    record.x[idx],
                    record.y[idx],
                    record.yaw[idx],
                    record.steer[idx],
                    vehicle_config,
                    color=color,
                    ax=axis,
                )

            axis.set_title("{} | All Controllers | t={:.1f}s".format(scenario.name, time_label))
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.axis("equal")
            axis.grid(True, alpha=0.3)
            axis.legend(frameon=False, ncol=3, loc="best")

        animation = FuncAnimation(
            figure,
            _draw_frame,
            frames=len(frame_indices),
            interval=60,
            repeat=False,
        )

        if output_path:
            animation.save(output_path, writer=PillowWriter(fps=15))

        if show:
            return figure, animation

        plt.close(figure)
        return None, None


def _save_individual_run_plots(results, history, scenarios, output_dir, vehicle_config, show=False):
    os.makedirs(output_dir, exist_ok=True)
    figures = []
    paths = []
    result_map = {(result.scenario, result.controller): result for result in results}

    for scenario in scenarios:
        controller_names = []
        for key_scenario, controller_name in history.keys():
            if key_scenario == scenario.name:
                controller_names.append(controller_name)

        for controller_name in controller_names:
            record = history[(scenario.name, controller_name)]
            result = result_map[(scenario.name, controller_name)]
            plot_path = os.path.join(
                output_dir,
                "{}_{}.png".format(_slugify(scenario.name), _slugify(controller_name)),
            )
            figure = plot_single_run(
                scenario,
                controller_name,
                result,
                record,
                plot_path,
                vehicle_config,
                show=show,
            )
            paths.append(plot_path)
            if figure is not None:
                figures.append(figure)

    return paths, figures


def _save_multi_controller_effects(
    scenarios,
    history,
    controller_names,
    output_dir,
    vehicle_config,
    make_animation=False,
    show_animation=False,
):
    os.makedirs(output_dir, exist_ok=True)
    snapshot_dir = os.path.join(output_dir, "snapshots")
    os.makedirs(snapshot_dir, exist_ok=True)
    animation_dir = os.path.join(output_dir, "animations")
    if make_animation:
        os.makedirs(animation_dir, exist_ok=True)

    snapshot_paths = []
    animation_paths = []
    figures = []
    animations = []

    for scenario in scenarios:
        snapshot_path = os.path.join(snapshot_dir, "{}_all_controllers.png".format(_slugify(scenario.name)))
        plot_multi_controller_snapshot(scenario, controller_names, history, snapshot_path, vehicle_config)
        snapshot_paths.append(snapshot_path)

        if make_animation or show_animation:
            animation_path = None
            if make_animation:
                animation_path = os.path.join(animation_dir, "{}_all_controllers.gif".format(_slugify(scenario.name)))

            figure, animation = animate_multi_controller_run(
                scenario,
                controller_names,
                history,
                output_path=animation_path,
                vehicle_config=vehicle_config,
                show=show_animation,
            )
            if animation_path is not None:
                animation_paths.append(animation_path)
            if figure is not None and animation is not None:
                figures.append(figure)
                animations.append(animation)

    return {
        "snapshot_dir": snapshot_dir,
        "animation_dir": animation_dir if make_animation else None,
        "snapshot_paths": snapshot_paths,
        "animation_paths": animation_paths,
        "figures": figures,
        "animations": animations,
    }


def _save_individual_run_animations(
    results,
    history,
    scenarios,
    output_dir,
    vehicle_config,
    make_animation=False,
    show_animation=False,
    frame_stride=2,
):
    if not make_animation and not show_animation:
        return [], [], []

    os.makedirs(output_dir, exist_ok=True)
    figures = []
    animations = []
    paths = []
    result_map = {(result.scenario, result.controller): result for result in results}

    for scenario in scenarios:
        controller_names = []
        for key_scenario, controller_name in history.keys():
            if key_scenario == scenario.name:
                controller_names.append(controller_name)

        for controller_name in controller_names:
            record = history[(scenario.name, controller_name)]
            result = result_map[(scenario.name, controller_name)]
            animation_path = None
            if make_animation:
                animation_path = os.path.join(
                    output_dir,
                    "{}_{}.gif".format(_slugify(scenario.name), _slugify(controller_name)),
                )
            figure, animation = animate_single_run(
                scenario,
                controller_name,
                result,
                record,
                output_path=animation_path,
                vehicle_config=vehicle_config,
                show=show_animation,
                frame_stride=frame_stride,
            )
            if animation_path is not None:
                paths.append(animation_path)
            if figure is not None and animation is not None:
                figures.append(figure)
                animations.append(animation)

    return paths, figures, animations


def build_compare_arg_parser():
    parser = argparse.ArgumentParser(description="Benchmark classical path tracking controllers.")
    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory used to save benchmark tables and figures.",
    )
    live_group = parser.add_mutually_exclusive_group()
    live_group.add_argument(
        "--live",
        dest="live",
        action="store_true",
        help="Play the car tracking process live in a popup window while simulating.",
    )
    live_group.add_argument(
        "--no-live",
        dest="live",
        action="store_false",
        help="Disable popup live simulation.",
    )
    parser.set_defaults(live=True)
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open per-controller plots after simulation finishes.",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Save GIF animations for every controller-task run.",
    )
    parser.add_argument(
        "--show-animation",
        action="store_true",
        help="Play generated animations in windows after simulation finishes.",
    )
    return parser


def build_controller_arg_parser(controller_name):
    parser = argparse.ArgumentParser(description="Run {} controller showcase.".format(controller_name))
    parser.add_argument(
        "--output-dir",
        default=os.path.join("outputs", _slugify(controller_name)),
        help="Directory used to save this controller's results.",
    )
    live_group = parser.add_mutually_exclusive_group()
    live_group.add_argument(
        "--live",
        dest="live",
        action="store_true",
        help="Play the car tracking process live in a popup window while simulating.",
    )
    live_group.add_argument(
        "--no-live",
        dest="live",
        action="store_false",
        help="Disable popup live simulation.",
    )
    parser.set_defaults(live=True)
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open per-scenario plots after simulation finishes.",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Save GIF animations for this controller.",
    )
    parser.add_argument(
        "--show-animation",
        action="store_true",
        help="Play generated animations for this controller.",
    )
    return parser


def run_full_benchmark(
    controllers,
    output_dir="outputs",
    show=False,
    make_animation=False,
    show_animation=False,
    live=False,
):
    _prepare_display_backend(live or show or show_animation)
    _reset_output_dir(output_dir)

    sim_config = SimulationConfig()
    vehicle_config = VehicleConfig()
    scenarios = build_scenarios(sim_config)

    results = []
    history = {}
    controller_names = [controller_name for controller_name, _ in controllers]

    for scenario in scenarios:
        for controller_name, controller_cls in controllers:
            result, record = run_benchmark(
                controller_name=controller_name,
                controller_cls=controller_cls,
                scenario=scenario,
                sim_config=sim_config,
                vehicle_config=vehicle_config,
                live_viewer=None,
            )
            results.append(result)
            history[(scenario.name, controller_name)] = record

    print(format_table(results))

    if live:
        play_multi_controller_live(scenarios, history, controller_names, vehicle_config)
    if not (show or show_animation):
        _switch_to_headless_backend()

    csv_path = os.path.join(output_dir, "benchmark_results.csv")
    md_path = os.path.join(output_dir, "benchmark_results.md")
    overview_path = os.path.join(output_dir, "trajectory_overview.png")
    dashboard_path = os.path.join(output_dir, "metric_dashboard.png")
    runs_dir = os.path.join(output_dir, "runs")
    animations_dir = os.path.join(output_dir, "animations")
    comparison_dir = os.path.join(output_dir, "comparison_effects")

    save_csv(results, csv_path)
    save_markdown_table(results, md_path)
    plot_trajectory_overview(scenarios, history, controller_names, overview_path, vehicle_config)
    plot_metric_dashboard(results, dashboard_path)
    run_paths, figures = _save_individual_run_plots(results, history, scenarios, runs_dir, vehicle_config, show=show)
    comparison_effects = _save_multi_controller_effects(
        scenarios,
        history,
        controller_names,
        comparison_dir,
        vehicle_config,
        make_animation=make_animation,
        show_animation=show_animation,
    )
    animation_paths, animation_figures, animations = _save_individual_run_animations(
        results,
        history,
        scenarios,
        animations_dir,
        vehicle_config,
        make_animation=make_animation,
        show_animation=show_animation,
    )

    if show and figures:
        plt.show()
        for figure in figures:
            plt.close(figure)

    if show_animation and animation_figures:
        plt.show()
        for figure in animation_figures:
            plt.close(figure)
    if show_animation and comparison_effects["figures"]:
        plt.show()
        for figure in comparison_effects["figures"]:
            plt.close(figure)

    artifact_paths = [csv_path, md_path, overview_path, dashboard_path, runs_dir, comparison_effects["snapshot_dir"]]
    if make_animation:
        artifact_paths.extend([animations_dir, comparison_effects["animation_dir"]])
    print("\nSaved artifacts:")
    for path in artifact_paths:
        print("  - {}".format(path))
    print("  - {} individual run plots".format(len(run_paths)))
    print("  - {} multi-controller effect snapshots".format(len(comparison_effects["snapshot_paths"])))
    if make_animation:
        print("  - {} animation GIFs".format(len(animation_paths)))
        print("  - {} multi-controller effect GIFs".format(len(comparison_effects["animation_paths"])))

    return {
        "results": results,
        "history": history,
        "paths": artifact_paths
        + run_paths
        + animation_paths
        + comparison_effects["snapshot_paths"]
        + comparison_effects["animation_paths"],
    }


def run_controller_suite(
    controller_name,
    controller_cls,
    output_dir,
    show=False,
    make_animation=False,
    show_animation=False,
    live=False,
):
    _prepare_display_backend(live or show or show_animation)
    _reset_output_dir(output_dir)

    sim_config = SimulationConfig()
    vehicle_config = VehicleConfig()
    scenarios = build_scenarios(sim_config)

    results = []
    history = {}
    live_viewer = LiveRunViewer(vehicle_config) if live else None

    for scenario in scenarios:
        result, record = run_benchmark(
            controller_name=controller_name,
            controller_cls=controller_cls,
            scenario=scenario,
            sim_config=sim_config,
            vehicle_config=vehicle_config,
            live_viewer=live_viewer,
        )
        results.append(result)
        history[(scenario.name, controller_name)] = record

    print(format_table(results))

    if live_viewer is not None:
        live_viewer.close()
    if not (show or show_animation):
        _switch_to_headless_backend()

    csv_path = os.path.join(output_dir, "{}_results.csv".format(_slugify(controller_name)))
    md_path = os.path.join(output_dir, "{}_results.md".format(_slugify(controller_name)))
    overview_path = os.path.join(output_dir, "{}_overview.png".format(_slugify(controller_name)))
    runs_dir = os.path.join(output_dir, "runs")
    animations_dir = os.path.join(output_dir, "animations")

    save_csv(results, csv_path)
    save_markdown_table(results, md_path)
    plot_controller_overview(scenarios, controller_name, history, overview_path, vehicle_config)
    run_paths, figures = _save_individual_run_plots(results, history, scenarios, runs_dir, vehicle_config, show=show)
    animation_paths, animation_figures, animations = _save_individual_run_animations(
        results,
        history,
        scenarios,
        animations_dir,
        vehicle_config,
        make_animation=make_animation,
        show_animation=show_animation,
    )

    if show and figures:
        plt.show()
        for figure in figures:
            plt.close(figure)

    if show_animation and animation_figures:
        plt.show()
        for figure in animation_figures:
            plt.close(figure)

    print("\nSaved artifacts:")
    print("  - {}".format(csv_path))
    print("  - {}".format(md_path))
    print("  - {}".format(overview_path))
    print("  - {}".format(runs_dir))
    print("  - {} scenario plots".format(len(run_paths)))
    if make_animation:
        print("  - {}".format(animations_dir))
        print("  - {} animation GIFs".format(len(animation_paths)))

    return {
        "results": results,
        "history": history,
        "paths": [csv_path, md_path, overview_path, runs_dir] + run_paths + animation_paths,
    }
