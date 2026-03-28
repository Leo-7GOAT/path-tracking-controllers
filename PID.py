from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp


class ControllerPurePID:
    name = "PID"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.speed_pid = PID1D(
            kp=1.0,
            ki=0.12,
            kd=0.035,
            integral_limit=5.0,
            integral_decay=0.995,
            sign_change_decay=0.25,
            error_deadband=0.03,
            derivative_smoothing=0.45,
        )
        self.lateral_pid = PID1D(kp=1.7, ki=0.05, kd=0.26, integral_limit=4.0)
        self.heading_gain = 1.7

    def control(self, state, ref_path, target_speed):
        index, lateral_error, heading_error, _ = ref_path.front_axle_error(
            state,
            self.vehicle_config,
            start_index=max(0, ref_path.last_index - 5),
        )

        accel = self.speed_pid.step(target_speed - state.v, self.dt)
        steer = self.lateral_pid.step(lateral_error, self.dt) + self.heading_gain * heading_error

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
    args = build_controller_arg_parser("PID").parse_args()
    run_controller_suite(
        controller_name="PID",
        controller_cls=ControllerPurePID,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
