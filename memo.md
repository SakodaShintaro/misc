# memo

## compile_commands.jsonを追加する

`~/.bashrc`に追加

```bash
export CMAKE_EXPORT_COMPILE_COMMANDS=1
```

## ビルドコマンドメモ

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

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
