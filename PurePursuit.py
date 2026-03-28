import math

from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp, normalize_angle


class PurePursuitController:
    name = "PurePursuit"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.min_lookahead = 2.0
        self.lookahead_gain = 0.28
        self.speed_pid = PID1D(
            kp=1.0,
            ki=0.1,
            kd=0.03,
            integral_limit=5.0,
            integral_decay=0.995,
            sign_change_decay=0.3,
            error_deadband=0.03,
            derivative_smoothing=0.45,
        )

    def _target_index(self, state, ref_path):
        nearest_index, _, _, _ = ref_path.tracking_error(
            state,
            start_index=max(0, ref_path.last_index - 5),
        )
        lookahead = self.min_lookahead + self.lookahead_gain * state.v
        target_s = ref_path.s[nearest_index] + lookahead
        target_index = nearest_index
        while target_index + 1 < ref_path.length and ref_path.s[target_index] < target_s:
            target_index += 1
        return target_index, lookahead

    def control(self, state, ref_path, target_speed):
        target_index, lookahead = self._target_index(state, ref_path)
        target_x = ref_path.cx[target_index]
        target_y = ref_path.cy[target_index]

        alpha = normalize_angle(math.atan2(target_y - state.y, target_x - state.x) - state.yaw)
        steer = math.atan2(2.0 * self.vehicle_config.wheel_base * math.sin(alpha), max(lookahead, 1e-6))
        accel = self.speed_pid.step(target_speed - state.v, self.dt)

        _, lateral_error, heading_error, _ = ref_path.tracking_error(
            state,
            start_index=max(0, ref_path.last_index - 5),
        )

        return (
            clamp(accel, -self.vehicle_config.max_accel, self.vehicle_config.max_accel),
            clamp(steer, -self.vehicle_config.max_steer, self.vehicle_config.max_steer),
            {
                "target_index": target_index,
                "lateral_error": lateral_error,
                "heading_error": heading_error,
            },
        )


def main():
    args = build_controller_arg_parser("PurePursuit").parse_args()
    run_controller_suite(
        controller_name="PurePursuit",
        controller_cls=PurePursuitController,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
