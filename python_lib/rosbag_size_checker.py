import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", help="rosbag(db3) directory", type=str)
    parser.add_argument(
        "--watch_topics",
        nargs="*",
        default=[
            "/perception/obstacle_segmentation/pointcloud",
            "/perception/occupancy_grid_map/map",
        ],
        help="Additional topics to display size information "
        "(default: obstacle_segmentation and occupancy_grid_map)",
    )
    return parser.parse_args()


def create_reader(bag_dir: str) -> SequentialReader:
    storage_options = StorageOptions(
        uri=bag_dir,
        storage_id="sqlite3",
    )
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_topic_stats(reader: SequentialReader) -> list:
    type_map = {}
    for topic_type in reader.get_all_topics_and_types():
        type_map[topic_type.name] = topic_type.type

    dict_size = defaultdict(int)
    dict_num = defaultdict(int)
    while reader.has_next():
        (topic, data, _) = reader.read_next()
        if topic != "/rosout":
            dict_size[topic] += np.int64(len(data))
            dict_num[topic] += 1

    # トピックごとの統計を計算
    topic_stats = []
    for topic in dict_size:
        total_size = dict_size[topic]
        num_messages = dict_num[topic]
        average_size = total_size / num_messages if num_messages > 0 else 0
        total_size_mb = total_size / (1024 * 1024)  # MB単位に変換

        topic_stats.append(
            {
                "topic": topic,
                "total_size_bytes": total_size,
                "total_size_mb": round(total_size_mb, 2),
                "num_messages": num_messages,
                "average_size_bytes": round(average_size, 2),
            },
        )

    # サイズの大きい順にソート
    topic_stats.sort(key=lambda x: x["total_size_bytes"], reverse=True)

    return topic_stats


def display_watch_topics(topic_stats: list, watch_topics: list) -> None:
    """指定されたトピックの詳細情報を表示する"""
    if not watch_topics:
        return

    print("\n=== Watch Topics Information ===")
    topic_dict = {stats["topic"]: stats for stats in topic_stats}

    found_topics = []
    missing_topics = []

    for topic in watch_topics:
        if topic in topic_dict:
            found_topics.append((topic, topic_dict[topic]))
        else:
            missing_topics.append(topic)

    if found_topics:
        print(f"{'Topic':<50} {'Size(MB)':<10} {'Messages':<10} {'Avg(bytes)':<12}")
        print("-" * 90)
        for topic, stats in found_topics:
            print(
                f"{topic:<50} {stats['total_size_mb']:<10} "
                f"{stats['num_messages']:<10} {stats['average_size_bytes']:<12}",
            )

    if missing_topics:
        print("\nMissing topics (not found in rosbag):")
        for topic in missing_topics:
            print(f"  - {topic}")


def vis_topic_size(stat: list, topic_vis_num: int) -> None:
    topic_label = [x[0] for x in stat]
    topic_size = [x[1] for x in stat]
    if len(topic_label) > topic_vis_num:
        other_topics_size = np.array(topic_size[topic_vis_num:]).sum()
        topic_label = topic_label[:topic_vis_num]
        topic_size = topic_size[:topic_vis_num]
        topic_label.append("others")
        topic_size.append(other_topics_size)

    total_size = int(np.array(topic_size).sum() / 1024**2)  # MiB

    plt.pie(
        topic_size,
        labels=topic_label,
        counterclock=False,
        startangle=90,
        wedgeprops={"linewidth": 1, "edgecolor": "white"},
        autopct="%1.0f%%",
        pctdistance=0.8,
    )
    plt.title("ROSbag Topic Size Distribution (total: " + str(total_size) + " MiB)")
    plt.show()


if __name__ == "__main__":
    args = parse_args()

    rosbag_dir = args.bag_dir

    assert Path(rosbag_dir).exists(), f"{rosbag_dir} does not exist."

    # rosbagを読み込んでトピック統計を取得
    reader = create_reader(rosbag_dir)
    topic_stats = get_topic_stats(reader)

    # 上位10個のトピックを表示
    print("=== ROSbag Topic Size Analysis ===")
    print(f"Total topics: {len(topic_stats)}")
    print()
    print("Top 10 largest topics:")
    print("-" * 80)
    print(f"{'Rank':<4} {'Topic':<40} {'Size(MB)':<10} {'Messages':<10} {'Avg(bytes)':<12}")
    print("-" * 80)

    for i, stats in enumerate(topic_stats[:10], 1):
        print(
            f"{i:<4} {stats['topic']:<40} {stats['total_size_mb']:<10} "
            f"{stats['num_messages']:<10} {stats['average_size_bytes']:<12}",
        )

    # 指定されたトピックの情報を表示
    display_watch_topics(topic_stats, args.watch_topics)

    # 結果をCSVファイルとして保存
    df = pd.DataFrame(topic_stats)
    save_path = Path(rosbag_dir).parent / "topic_size_analysis.csv"
    df.to_csv(save_path, index=False)
    print(f"\nFull analysis saved to: {save_path}")

    # 簡単な円グラフも表示
    if len(topic_stats) > 0:
        vis_topic_size([(stats["topic"], stats["total_size_bytes"]) for stats in topic_stats], 10)
