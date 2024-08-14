import sqlite3
import argparse
from pathlib import Path
from rosbag2_py import Reindexer, StorageOptions


def parse_args():
    parser = argparse.ArgumentParser(
        description="Delete topics with zero messages from a rosbag database."
    )
    parser.add_argument("rosbag_db3_file", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_db3_file = args.rosbag_db3_file

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
    """
    )
    topics_to_delete = cursor.fetchall()

    for topic_id, topic_name in topics_to_delete:
        print(f"Deleting topic: {topic_name}")
        cursor.execute("DELETE FROM topics WHERE id = ?", (topic_id,))
        cursor.execute("DELETE FROM messages WHERE topic_id = ?", (topic_id,))

    conn.commit()
    conn.close()

    print("Completed deleting topics with zero messages.")
    storage_options = StorageOptions(
        uri=str(rosbag_db3_file.parent),
        storage_id="sqlite3",
    )
    Reindexer().reindex(storage_options)
