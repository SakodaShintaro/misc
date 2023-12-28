# memo

## compile_commands.jsonを追加する

`~/.bashrc`に追加

```bash
export CMAKE_EXPORT_COMPILE_COMMANDS=1
```

## ビルドコマンドメモ

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to

## バージョン出力

vcs export src --exact > my_autoware.repos

## cspell

```bash
sudo apt install -y nodejs npm
sudo npm install n -g
sudo n stable
sudo npm install -g cspell
cspell "**/*"
cspell "**/*.{hpp,cpp,md}"
```

## mcap

<https://proc-cpuinfo.fixstars.com/2023/01/rosbag2_storage_mcap_usage/>

```bash
sudo apt install -y ros-humble-rosbag2-storage-mcap
ros2 bag record -s mcap --all
```

## gitのエディタをvimにする

```bash
git config --global core.editor "vim"
```

## Dockerメモ

```bash
docker system df
docker builder prune
docker system prune -a
```

## AWSIMで信号を無視する方法

autoware/src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
の3行目あたりにある変数input_traffic_light_topic_nameを適当に書き換える（suffixに_dummyと付けるなど）
