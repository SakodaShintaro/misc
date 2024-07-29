"""Driving Log Replayerのscenario.yamlの検証を行うスクリプト"""

import argparse
from pathlib import Path
import yaml
import os


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_yaml", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    scenario_yaml = args.scenario_yaml
    print(scenario_yaml)

    with scenario_yaml.open() as f:
        scenario = yaml.safe_load(f)

    local_map_path = Path(os.path.expandvars(scenario["LocalMapPath"]))
    assert local_map_path.exists()

    name = local_map_path.parent.name
    scenario["ScenarioName"] = name
    scenario["ScenarioDescription"] = name

    scenario["SensorModel"] = "sample_sensor_kit"
    scenario["VehicleModel"] = "sample_vehicle"
    scenario["VehicleId"] = "default"

    scenario["Evaluation"]["Conditions"]["Convergence"]["PassRate"] = 90

    with scenario_yaml.open("w") as f:
        yaml.safe_dump(scenario, f, sort_keys=False)
