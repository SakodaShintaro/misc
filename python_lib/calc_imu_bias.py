"""(静止状態であるrosbagと前提して)IMUのバイアスを計算するスクリプト."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt

from parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--target_topic", type=str, default="/sensing/imu/imu_data")
    parser.add_argument("--start_time_from_zero", action="store_true")
    parser.add_argument("--time_begin_sec", type=float, default=0.0)
    parser.add_argument("--time_end_sec", type=float, default=float("inf"))
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = args.target_topic
    start_time_from_zero = args.start_time_from_zero
    time_begin_sec = args.time_begin_sec
    time_end_sec = args.time_end_sec

    df_dict = parse_rosbag(str(rosbag_path), [target_topic])

    # rosbag path may be the path to the db3 file, or may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / target_topic.split("/")[-1]
    save_dir.mkdir(exist_ok=True)

    # save as csv
    for topic_name in [target_topic]:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        filename = topic_name.replace("/sensing/", "").replace("/", "__")
        df.to_csv(save_dir / f"{filename}.tsv", index=False, sep="\t", float_format="%.9f")
    df = df_dict[target_topic]

    """
    print(df.head())
                timestamp           frame_id  angular_velocity.x  angular_velocity.y  angular_velocity.z  linear_acceleration.x  linear_acceleration.y  linear_acceleration.z  orientation.x  orientation.y  orientation.z  orientation.w
    0  1721370795397252788  tamagawa/imu_link            0.000426           -0.002344            0.009800               0.061035              -0.140381              -9.713745            0.0            0.0            0.0            1.0
    1  1721370795402274208  tamagawa/imu_link           -0.000107           -0.002024            0.008203               0.067139              -0.106812              -9.844971            0.0            0.0            0.0            1.0
    2  1721370795407268850  tamagawa/imu_link           -0.001917           -0.002663            0.007883               0.027466              -0.131226              -9.698486            0.0            0.0            0.0            1.0
    3  1721370795412342401  tamagawa/imu_link           -0.001278           -0.001278            0.007776               0.042725              -0.085449              -9.796143            0.0            0.0            0.0            1.0
    4  1721370795417306159  tamagawa/imu_link           -0.000639           -0.002131            0.008629               0.070190              -0.119019              -9.768677            0.0            0.0            0.0            1.0
    """  # noqa: E501

    if start_time_from_zero:
        df["timestamp"] -= df["timestamp"].iloc[0]
    df["timestamp"] /= 1e9  # ns -> s
    df = df[(time_begin_sec <= df["timestamp"]) & (df["timestamp"] <= time_end_sec)]

    print(f"{df['angular_velocity.x'].mean()=:+.6f}")
    print(f"{df['angular_velocity.y'].mean()=:+.6f}")
    print(f"{df['angular_velocity.z'].mean()=:+.6f}")

    # smoothing
    df["angular_velocity.x"] = df["angular_velocity.x"].rolling(window=100).mean()
    df["angular_velocity.y"] = df["angular_velocity.y"].rolling(window=100).mean()
    df["angular_velocity.z"] = df["angular_velocity.z"].rolling(window=100).mean()

    # plot
    plt.plot(df["timestamp"], df["angular_velocity.x"], label="angular_velocity.x")
    plt.plot(df["timestamp"], df["angular_velocity.y"], label="angular_velocity.y")
    plt.plot(df["timestamp"], df["angular_velocity.z"], label="angular_velocity.z")
    plt.legend()
    plt.grid()
    plt.title(target_topic)
    plt.xlabel("timestamp")
    plt.ylabel("angular_velocity[rad/s]")
    save_path = save_dir / "angular_velocity.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Save {save_path}")
