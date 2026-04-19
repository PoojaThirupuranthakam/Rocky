"""Train/evaluate the model-based navigator explicitly."""

import argparse
import os
import sys

sys.path.append(os.path.dirname(__file__))

from ai_navigator_final import AINavigator

INCH_TO_CM = 2.54


def parse_args():
    parser = argparse.ArgumentParser(description="Train cave robot navigation model")
    parser.add_argument(
        "--model-path",
        default="models/navigation_model.pkl",
        help="Output model path",
    )
    parser.add_argument(
        "--retrain",
        action="store_true",
        help="Delete existing model first and retrain from scratch",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=16000,
        help="Total synthetic samples to generate for supervised training",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=220,
        help="Number of training epochs",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=0.03,
        help="Learning rate",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=256,
        help="Training batch size",
    )
    parser.add_argument(
        "--teacher",
        choices=["rules", "planner_rollout"],
        default="planner_rollout",
        help="Training target source: handcrafted rules or planner rollout imitation",
    )
    parser.add_argument(
        "--rollout-max-steps",
        type=int,
        default=280,
        help="Max steps per planner rollout episode when --teacher planner_rollout",
    )
    parser.add_argument(
        "--collision-weight",
        type=float,
        default=1.8,
        help="Extra sample weight for collision frames during training",
    )
    parser.add_argument(
        "--near-wall-weight",
        type=float,
        default=1.4,
        help="Extra sample weight when front/side clearances are tight",
    )
    parser.add_argument(
        "--hard-ratio",
        type=float,
        default=0.45,
        help="Fraction of hard high-complexity rollout episodes (0..1)",
    )
    parser.add_argument(
        "--cm-per-pixel",
        type=float,
        default=0.5,
        help="World scaling used for robot/person size to pixel conversion",
    )
    parser.add_argument(
        "--robot-length-cm",
        type=float,
        default=7.0 * INCH_TO_CM,
        help="Robot length in centimeters (default = 7 inches)",
    )
    parser.add_argument(
        "--robot-width-cm",
        type=float,
        default=5.0 * INCH_TO_CM,
        help="Robot width in centimeters (default = 5 inches)",
    )
    parser.add_argument(
        "--robot-height-cm",
        type=float,
        default=5.0 * INCH_TO_CM,
        help="Robot height in centimeters (default = 5 inches)",
    )
    parser.add_argument(
        "--person-width-min-cm",
        type=float,
        default=28.0,
        help="Minimum person width in cm for size-diverse rollout training (children)",
    )
    parser.add_argument(
        "--person-width-max-cm",
        type=float,
        default=56.0,
        help="Maximum person width in cm for size-diverse rollout training (adults)",
    )
    parser.add_argument(
        "--person-height-min-cm",
        type=float,
        default=100.0,
        help="Minimum person height in cm for size-diverse rollout training (children)",
    )
    parser.add_argument(
        "--person-height-max-cm",
        type=float,
        default=190.0,
        help="Maximum person height in cm for size-diverse rollout training (adults)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    model_path = args.model_path
    if args.retrain and os.path.exists(model_path):
        os.remove(model_path)
        print(f"♻️ Removed existing model: {model_path}")

    training_config = {
        'samples': max(1000, int(args.samples)),
        'epochs': max(1, int(args.epochs)),
        'lr': max(1e-5, float(args.lr)),
        'batch_size': max(16, int(args.batch_size)),
        'teacher': args.teacher,
        'rollout_max_steps': max(40, int(args.rollout_max_steps)),
        'collision_weight': max(1.0, float(args.collision_weight)),
        'near_wall_weight': max(1.0, float(args.near_wall_weight)),
        'hard_ratio': min(1.0, max(0.0, float(args.hard_ratio))),
        'cm_per_pixel': max(0.1, float(args.cm_per_pixel)),
        'robot_length_cm': max(1.0, float(args.robot_length_cm)),
        'robot_width_cm': max(1.0, float(args.robot_width_cm)),
        'robot_height_cm': max(1.0, float(args.robot_height_cm)),
        'person_width_min_cm': max(20.0, float(args.person_width_min_cm)),
        'person_width_max_cm': max(20.0, float(args.person_width_max_cm)),
        'person_height_min_cm': max(80.0, float(args.person_height_min_cm)),
        'person_height_max_cm': max(80.0, float(args.person_height_max_cm)),
    }

    # Normalize ranges if user passes them in reverse order.
    if training_config['person_width_min_cm'] > training_config['person_width_max_cm']:
        training_config['person_width_min_cm'], training_config['person_width_max_cm'] = (
            training_config['person_width_max_cm'],
            training_config['person_width_min_cm'],
        )
    if training_config['person_height_min_cm'] > training_config['person_height_max_cm']:
        training_config['person_height_min_cm'], training_config['person_height_max_cm'] = (
            training_config['person_height_max_cm'],
            training_config['person_height_min_cm'],
        )

    navigator = AINavigator(model_path=model_path, training_config=training_config)
    print("\n📊 Training summary")
    if navigator.training_metrics:
        for k, v in navigator.training_metrics.items():
            print(f"- {k}: {v}")
    else:
        print("- No training metrics found (loaded older model format).")


if __name__ == "__main__":
    main()
