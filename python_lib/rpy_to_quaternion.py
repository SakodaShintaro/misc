import argparse

from scipy.spatial.transform import Rotation as R


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("roll", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("yaw", type=float)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(f"Roll: {args.roll}, Pitch: {args.pitch}, Yaw: {args.yaw}")
    r = R.from_euler("xyz", [args.roll, args.pitch, args.yaw], degrees=True)
    q = r.as_quat()
    print(f"x: {q[0]}")
    print(f"y: {q[1]}")
    print(f"z: {q[2]}")
    print(f"w: {q[3]}")
