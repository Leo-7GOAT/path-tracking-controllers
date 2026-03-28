import math

import numpy as np

from benchmark_runner import build_controller_arg_parser, run_controller_suite
from common import PID1D, clamp, normalize_angle


class MPCController:
    name = "MPC"

    def __init__(self, dt=0.1, vehicle_config=None):
        self.dt = dt
        self.vehicle_config = vehicle_config
        self.horizon = 16
        self.Q = np.diag([5.0, 0.45, 4.2, 0.3])
        self.Qf = np.diag([8.5, 0.8, 7.2, 0.5])
        self.R_input = 0.45
        self.R_rate = 1.8
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

    def _build_prediction_matrices(self, A, B):
        state_dim = A.shape[0]
        horizon = self.horizon
        s_x = np.zeros((horizon * state_dim, state_dim), dtype=float)
        s_u = np.zeros((horizon * state_dim, horizon), dtype=float)

        a_powers = [np.eye(state_dim, dtype=float)]
        for _ in range(horizon):
            a_powers.append(a_powers[-1].dot(A))

        for step in range(horizon):
            row = slice(step * state_dim, (step + 1) * state_dim)
            s_x[row, :] = a_powers[step + 1]
            for control_step in range(step + 1):
                col = slice(control_step, control_step + 1)
                s_u[row, col] = a_powers[step - control_step].dot(B)

        return s_x, s_u

    def _difference_operator(self):
        diff = np.eye(self.horizon, dtype=float)
        for i in range(1, self.horizon):
            diff[i, i - 1] = -1.0
        return diff

    def _solve_mpc(self, A, B, state_vector):
        s_x, s_u = self._build_prediction_matrices(A, B)
        q_bar = np.zeros((self.horizon * 4, self.horizon * 4), dtype=float)
        for i in range(self.horizon - 1):
            row = slice(i * 4, (i + 1) * 4)
            q_bar[row, row] = self.Q
        q_bar[-4:, -4:] = self.Qf

        r_bar = self.R_input * np.eye(self.horizon, dtype=float)
        diff = self._difference_operator()
        prev_input = np.zeros((self.horizon, 1), dtype=float)
        prev_input[0, 0] = self.prev_feedback_steer

        hessian = s_u.T.dot(q_bar).dot(s_u) + r_bar + self.R_rate * diff.T.dot(diff)
        gradient = s_u.T.dot(q_bar).dot(s_x).dot(state_vector) - self.R_rate * diff.T.dot(prev_input)
        regularization = 1e-6 * np.eye(self.horizon, dtype=float)

        try:
            control_sequence = -np.linalg.solve(hessian + regularization, gradient)
        except np.linalg.LinAlgError:
            control_sequence = -np.linalg.lstsq(hessian + regularization, gradient, rcond=None)[0]

        return control_sequence.flatten()

    def control(self, state, ref_path, target_speed):
        index, lateral_error, heading_error, curvature = ref_path.tracking_error(
            state,
            start_index=max(0, ref_path.last_index - 5),
        )
        lateral_rate = (lateral_error - self.prev_lateral_error) / self.dt
        heading_rate = normalize_angle(heading_error - self.prev_heading_error) / self.dt

        model_speed = max(abs(state.v), abs(target_speed), 2.0)
        A, B = self._discrete_model(model_speed)
        state_vector = np.array(
            [[lateral_error], [lateral_rate], [heading_error], [heading_rate]],
            dtype=float,
        )
        control_sequence = self._solve_mpc(A, B, state_vector)
        feedback_steer = clamp(
            float(control_sequence[0]),
            -0.9 * self.vehicle_config.max_steer,
            0.9 * self.vehicle_config.max_steer,
        )

        feedforward_steer = math.atan(self.vehicle_config.wheel_base * curvature)
        steer = feedforward_steer + feedback_steer
        accel = self.speed_pid.step(target_speed - state.v, self.dt)

        self.prev_lateral_error = lateral_error
        self.prev_heading_error = heading_error
        self.prev_feedback_steer = feedback_steer

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
