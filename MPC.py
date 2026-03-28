import math

import numpy as np

from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp, normalize_angle


class MPCController:
    name = "MPC"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.horizon = 12
        self.Q = np.diag([4.5, 0.35, 3.5, 0.25])
        self.Qf = np.diag([8.0, 0.6, 6.5, 0.4])
        self.R_input = 0.35
        self.R_rate = np.diag([1.25])
        self.speed_pid = PID1D(
            kp=1.0,
            ki=0.12,
            kd=0.035,
            integral_limit=5.0,
            integral_decay=0.995,
            sign_change_decay=0.3,
            error_deadband=0.03,
            derivative_smoothing=0.45,
        )
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0
        self.prev_feedback_steer = 0.0

    def _discrete_model(self, speed):
        speed = max(1.2, abs(speed))
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

    def _riccati_gains(self, A, B):
        A_aug = np.block(
            [
                [A, B],
                [np.zeros((1, 4), dtype=float), np.ones((1, 1), dtype=float)],
            ]
        )
        B_aug = np.vstack([B, np.ones((1, 1), dtype=float)])

        Q_aug = np.zeros((5, 5), dtype=float)
        Q_aug[:4, :4] = self.Q
        Q_aug[4, 4] = self.R_input

        Qf_aug = np.zeros((5, 5), dtype=float)
        Qf_aug[:4, :4] = self.Qf
        Qf_aug[4, 4] = self.R_input

        gains = []
        P = Qf_aug.copy()
        for _ in range(self.horizon):
            S = self.R_rate + B_aug.T.dot(P).dot(B_aug)
            K = np.linalg.solve(S, B_aug.T.dot(P).dot(A_aug))
            gains.append(K)
            P = Q_aug + A_aug.T.dot(P).dot(A_aug - B_aug.dot(K))
        gains.reverse()
        return gains

    def control(self, state, ref_path, target_speed):
        index, lateral_error, heading_error, curvature = ref_path.tracking_error(
            state,
            start_index=max(0, ref_path.last_index - 5),
        )
        lateral_rate = (lateral_error - self.prev_lateral_error) / self.dt
        heading_rate = normalize_angle(heading_error - self.prev_heading_error) / self.dt

        A, B = self._discrete_model(state.v)
        gains = self._riccati_gains(A, B)

        augmented_state = np.array(
            [
                [lateral_error],
                [lateral_rate],
                [heading_error],
                [heading_rate],
                [self.prev_feedback_steer],
            ],
            dtype=float,
        )
        delta_feedback = self.prev_feedback_steer - float(gains[0].dot(augmented_state))
        delta_feedback = clamp(
            delta_feedback,
            -0.9 * self.vehicle_config.max_steer,
            0.9 * self.vehicle_config.max_steer,
        )

        delta_feedforward = math.atan(self.vehicle_config.wheel_base * curvature)
        steer = delta_feedforward + delta_feedback
        accel = self.speed_pid.step(target_speed - state.v, self.dt)

        self.prev_lateral_error = lateral_error
        self.prev_heading_error = heading_error
        self.prev_feedback_steer = delta_feedback

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
    args = build_controller_arg_parser("MPC").parse_args()
    run_controller_suite(
        controller_name="MPC",
        controller_cls=MPCController,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
