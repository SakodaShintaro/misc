# memo

## compile_commands.jsonを追加する

`~/.bashrc`に追加

```bash
export CMAKE_EXPORT_COMPILE_COMMANDS=1
```

## ビルドコマンドメモ

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to

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

## サブディレクトリを1階層分だけ

```bash
find target_dir -mindepth 1 -maxdepth 1 | sort | xargs -Ipath echo path
```
