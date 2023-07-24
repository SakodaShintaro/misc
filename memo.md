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
