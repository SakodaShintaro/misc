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

    df_dict = parse_rosbag(
        str(rosbag_path),
        ["/control/command/turn_indicators_cmd", "/vehicle/status/turn_indicators_status"],
    )

    # rosbag path may be the path to the db3 file, or may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "turn_indicators"
    save_dir.mkdir(exist_ok=True)

    # save as csv
    for topic_name in [
        "/control/command/turn_indicators_cmd",
        "/vehicle/status/turn_indicators_status",
    ]:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        filename = (
            topic_name.replace("/control/command/", "")
            .replace("/vehicle/status/", "")
            .replace("/", "__")
        )
        df.to_csv(save_dir / f"{filename}.tsv", index=False, sep="\t", float_format="%.9f")

    df_output = df_dict["/control/command/turn_indicators_cmd"]
    df_status = df_dict["/vehicle/status/turn_indicators_status"]

    print(df_output.head())
    print(df_status.head())
