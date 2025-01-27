"""指定したディレクトリ以下にあるlaunch.logを再帰的に探し、中身について特定の文字列を検索するスクリプト."""

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    parser.add_argument("keyword", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir
    keyword = args.keyword

    for log_file in target_dir.glob("**/launch.log"):
        with log_file.open() as f:
            if keyword in f.read():
                print(log_file)
