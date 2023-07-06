# Git merged memo

Gitでマージ済みブランチを削除するときに行うコマンドについて

ちょっと怖い作業なので目視で確認しながらやっていく

## ローカルについて確認&削除

git branch --merged
git branch -d <branch名>

## リモートについて確認&削除

git branch -r --merged
git push --delete origin <branch名>
