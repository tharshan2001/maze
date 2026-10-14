"""Train a DQN agent to reach random goals in the Gazebo maze.

Start the simulation headless first (the env does not launch it):

    ros2 launch maze_gazebo maze.launch.py headless:=true
    ros2 run maze_dqn train --timesteps 60000 --out models/dqn_maze

The Gazebo bridge only runs one simulator, so training speed is bounded by
real-time. Use `--rtf` guidance in the report and keep timesteps moderate.
"""
import argparse
import os

from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from maze_dqn.dqn_env import MazeNavEnv


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--timesteps", type=int, default=60000)
    p.add_argument("--out", type=str, default="models/dqn_maze")
    p.add_argument("--control-period", type=float, default=0.2)
    p.add_argument("--max-steps", type=int, default=400)
    p.add_argument("--seed", type=int, default=1)
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    env = MazeNavEnv(
        control_period=args.control_period,
        max_steps=args.max_steps,
        random_goal=True,
    )
    env = Monitor(env)

    model = DQN(
        "MlpPolicy",
        env,
        learning_rate=1e-3,
        buffer_size=100_000,
        learning_starts=2_000,
        batch_size=64,
        gamma=0.99,
        train_freq=4,
        target_update_interval=1_000,
        exploration_fraction=0.4,
        exploration_final_eps=0.05,
        policy_kwargs=dict(net_arch=[256, 256]),
        tensorboard_log="./runs/",
        seed=args.seed,
        verbose=1,
    )

    checkpoint = CheckpointCallback(
        save_freq=10_000, save_path="models/checkpoints", name_prefix="dqn_maze"
    )
    model.learn(total_timesteps=args.timesteps, callback=checkpoint)
    model.save(args.out)
    print(f"Saved model to {args.out}.zip")

    env.close()


if __name__ == "__main__":
    main()
