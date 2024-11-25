"""A script to delete topics with zero messages from a rosbag database."""

import argparse
import sqlite3
from pathlib import Path

from rosbag2_py import Reindexer, StorageOptions


def delete_topics_with_zero_messages(rosbag_db3_file: Path) -> None:
    conn = sqlite3.connect(rosbag_db3_file)
    cursor = conn.cursor()

    cursor.execute(
        """
        SELECT topics.id, topics.name
        FROM topics
        LEFT JOIN messages
        ON topics.id = messages.topic_id
        GROUP BY topics.id
        HAVING COUNT(messages.topic_id) = 0
        """,
    )
    topics_to_delete = cursor.fetchall()

    for topic_id, topic_name in topics_to_delete:
        print(f"Deleting topic: {topic_name}")
        cursor.execute("DELETE FROM topics WHERE id = ?", (topic_id,))
        cursor.execute("DELETE FROM messages WHERE topic_id = ?", (topic_id,))

    conn.commit()
    conn.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_db3_file", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_db3_file = args.rosbag_db3_file

    if rosbag_db3_file.is_dir():
        rosbag_db3_file_list = list(rosbag_db3_file.glob("*.db3"))
        for rosbag_db3_file in rosbag_db3_file_list:
            delete_topics_with_zero_messages(rosbag_db3_file)
    else:
        delete_topics_with_zero_messages(rosbag_db3_file)

    print("Completed deleting topics with zero messages.")
    storage_options = StorageOptions(
        uri=str(rosbag_db3_file.parent),
        storage_id="sqlite3",
    )
    Reindexer().reindex(storage_options)
