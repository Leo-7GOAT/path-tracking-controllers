from benchmark_runner import build_compare_arg_parser, run_full_benchmark
from LQR import LQRPathTrackingController
from MPC import MPCController
from PID import ControllerPurePID
from PurePursuit import PurePursuitController
from Stanley import StanleyController


CONTROLLERS = [
    ("Stanley", StanleyController),
    ("PurePursuit", PurePursuitController),
    ("PID", ControllerPurePID),
    ("LQR", LQRPathTrackingController),
    ("MPC", MPCController),
]


def main():
    args = build_compare_arg_parser().parse_args()
    run_full_benchmark(
        controllers=CONTROLLERS,
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
        live=args.live,
    )


if __name__ == "__main__":
    main()
