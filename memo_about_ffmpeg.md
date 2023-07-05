# Memo about ffmpeg

## 動画作成メモ

cwdで設定しておくと便利

```python
subprocess.run("ffmpeg -r 5 -f image2 -i color_20000_%03d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p ../output.mp4", shell=True, cwd=path_to_output_dir)
```

## 動画作成

```bash
ffmpeg -y -r 10 -pattern_type glob -i “*.png” -pix_fmt yuv420p -vcodec libx264 -vf “scale=trunc(iw/2)*2:trunc(ih/2)*2” -crf 30 output.mp4
```

## 倍速

```bash
ffmpeg -i input.mp4 -vf setpts=PTS/20 -an output_fast.mp4
```

## webm to mp4

```bash
ffmpeg -i filename.webm filename.mp4
ffmpeg -i filename.mp4 -vf scale=-1:540 -crf 32 output.mp4
ffmpeg -i filename.mp4 -vf scale=-1:540 -vf setpts=PTS/2 -crf 32 output.mp4
```
