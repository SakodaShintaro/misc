import argparse
from pathlib import Path

from parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path

    target_topics = [
        "/planning/trajectory_generator/diffusion_planner_node/debug/processing_time_detail_ms",
    ]

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    # rosbag path may be the path to the db3 file, or may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "dp_result"
    save_dir.mkdir(exist_ok=True)

    # save as tsv
    for topic_name in target_topics:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        filename = topic_name.replace("/planning/", "").replace("/", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    df = df_dict[
        "/planning/trajectory_generator/diffusion_planner_node/debug/processing_time_detail_ms"
    ]

    # NaNがある行を削除
    df = df.dropna()

    with (save_dir / "processing_time_detail.txt").open("w") as f:
        f.write(f"{len(df)} msgs\n")
        for column in df.columns:
            mean = df[column].mean()
            std = df[column].std()
            print(f"{column}: {mean:.1f} ms (± {std:.1f} ms)")
            f.write(f"{column}: {mean:.1f} ms (± {std:.1f} ms)\n")
