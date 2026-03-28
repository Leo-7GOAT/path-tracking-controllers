import math

from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp


class StanleyController:
    name = "Stanley"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.k = 2.1
        self.softening = 1.2
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

    def control(self, state, ref_path, target_speed):
        index, lateral_error, heading_error, _ = ref_path.front_axle_error(
            state,
            self.vehicle_config,
            start_index=max(0, ref_path.last_index - 5),
        )

        steer = heading_error + math.atan2(self.k * lateral_error, self.softening + abs(state.v))
        accel = self.speed_pid.step(target_speed - state.v, self.dt)

        return (
            clamp(accel, -self.vehicle_config.max_accel, self.vehicle_config.max_accel),
            clamp(steer, -self.vehicle_config.max_steer, self.vehicle_config.max_steer),
            {
                "target_index": index,
                "lateral_error": lateral_error,
                "heading_error": heading_error,
            },
        )


def main():
    args = build_controller_arg_parser("Stanley").parse_args()
    run_controller_suite(
        controller_name="Stanley",
        controller_cls=StanleyController,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
