import math

import numpy as np

from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp, normalize_angle


class LQRPathTrackingController:
    name = "LQR"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.Q = np.diag([3.2, 0.25, 2.6, 0.18])
        self.R = np.diag([0.85])
        self.speed_pid = PID1D(
            kp=1.0,
            ki=0.12,
            kd=0.03,
            integral_limit=5.0,
            integral_decay=0.995,
            sign_change_decay=0.3,
            error_deadband=0.03,
            derivative_smoothing=0.45,
        )
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0

    def _discrete_model(self, speed):
        speed = max(1.0, abs(speed))
        A = np.array(
            [
                [1.0, self.dt, 0.0, 0.0],
                [0.0, 0.0, speed, 0.0],
                [0.0, 0.0, 1.0, self.dt],
                [0.0, 0.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        B = np.array([[0.0], [0.0], [0.0], [speed / self.vehicle_config.wheel_base]], dtype=float)
        return A, B

    def _solve_dare(self, A, B):
        P = self.Q.copy()
        for _ in range(150):
            bt_p = B.T.dot(P)
            gain_term = np.linalg.solve(self.R + bt_p.dot(B), bt_p.dot(A))
            P_next = self.Q + A.T.dot(P).dot(A - B.dot(gain_term))
            if np.max(np.abs(P_next - P)) < 1e-6:
                P = P_next
                break
            P = P_next
        return P

    def control(self, state, ref_path, target_speed):
        index, lateral_error, heading_error, curvature = ref_path.tracking_error(
            state,
            start_index=max(0, ref_path.last_index - 5),
        )
        lateral_rate = (lateral_error - self.prev_lateral_error) / self.dt
        heading_rate = normalize_angle(heading_error - self.prev_heading_error) / self.dt

        state_vector = np.array(
            [[lateral_error], [lateral_rate], [heading_error], [heading_rate]],
            dtype=float,
        )
        model_speed = max(abs(state.v), abs(target_speed), 2.0)
        A, B = self._discrete_model(model_speed)
        P = self._solve_dare(A, B)
        K = np.linalg.solve(self.R + B.T.dot(P).dot(B), B.T.dot(P).dot(A))

        steer_ff = math.atan(self.vehicle_config.wheel_base * curvature)
        steer_fb = -float(K.dot(state_vector))
        steer = steer_ff + steer_fb
        accel = self.speed_pid.step(target_speed - state.v, self.dt)

        self.prev_lateral_error = lateral_error
        self.prev_heading_error = heading_error

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
    args = build_controller_arg_parser("LQR").parse_args()
    run_controller_suite(
        controller_name="LQR",
        controller_cls=LQRPathTrackingController,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
