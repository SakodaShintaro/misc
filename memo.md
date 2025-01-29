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

## ファイル名をユニークに変更

```bash
find target_dir -name "*.png" | sort | xargs -I{} sh -c 'dir=$(dirname "{}"); prefix=$(basename "$dir"); orig=$(basename "{}"); echo "${dir}/${prefix}_${orig}"'
```

## シェルスクリプトで実行時間の取得

```bash
# set -euxをしているので
elapsed_time=$(date -u -d "@$SECONDS" +"%T")
# をするだけで表示される
```

## diagnostic_graph_monitor

```bash
ros2 run rqt_diagnostic_graph_monitor rqt_diagnostic_graph_monitor
```

## マウス速度

```bash
xinput list
xinput list-props 11
xinput --set-prop 11 "libinput Accel Speed" -0.2
# 必要であれば~/.bashrcに追加する
```

## 自動リブート処理とデーモン起動のためのコマンドスクリプト

<https://nxmnpg.lemoda.net/ja/8/rc.local>

## swapの積極性変更

```bash
sudo vim /etc/sysctl.conf
vm.swappiness=10
sudo sysctl -p
```

## GitHubへのアクセスをsshで行う

`~/.gitconfig` 下に以下を追記する

```bash
[url "git@github.com:"]
  insteadOf = https://github.com/
```

## CAPS LOCKキー(Aキーの左のキー)に、Controlキーの機能を割り当てる方法

/etc/default/keyboard に以下の設定を行う。

```txt
sudo vim /etc/default/keyboard
XKBOPTIONS="ctrl:nocaps"

sudo dpkg-reconfigure keyboard-configuration
sudo udevadm trigger --subsystem-match=input --action=change

sudo reboot
```
