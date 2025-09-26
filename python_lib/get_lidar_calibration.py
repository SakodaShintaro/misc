"""tf_staticを読み込んでbase_link -> hesai_topのPoseを得るスクリプト"""

import argparse
from pathlib import Path

from builtin_interfaces.msg import Time
from tf2_ros import Buffer

from parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    path_to_rosbag = args.path_to_rosbag

    target_topics = ["/tf_static"]

    df_dict = parse_rosbag(str(path_to_rosbag), target_topics)

    df_tf_static = df_dict["/tf_static"]
    tf_buffer = Buffer()
    for _, row in df_tf_static.iterrows():
        for transform_stamped in row["transforms"]:
            tf_buffer.set_transform_static(transform_stamped, "default_authority")

    transform = tf_buffer.lookup_transform(
        target_frame="base_link",
        source_frame="hesai_top",
        time=Time(),
    )
    print(transform)
