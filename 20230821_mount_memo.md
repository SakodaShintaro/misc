# SSDの名前を変更したときのメモ

```bash
# 確認
df -h
mount | grep /dev/sda2

# 名前変更のために一度unmount
sudo umount /dev/sda2

# 変更のためのツール導入と変更
sudo apt install exfatprogs
sudo exfatlabel /dev/sda2 ssd_1t
```
