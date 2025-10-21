#!/bin/bash
#=================================================
# lichtblick 自動録画スクリプト
#
# 使い方:
# ./record_lichtblick.sh [mcapファイル]
#=================================================

# --- クラス名 ---
TARGET_CLASSNAME="Lichtblick"

# --- 1. 引数のチェック ---
if [ "$#" -ne 1 ]; then
    echo "エラー: 引数が正しくありません。"
    echo "使い方: $0 [mcapファイル]"
    exit 1
fi

MCAP_FILE="$1"

# --- 2. 依存ツールのチェック ---
command -v ffmpeg >/dev/null 2>&1 || { echo >&2 "ffmpeg が見つかりません。"; exit 1; }
command -v xdotool >/dev/null 2>&1 || { echo >&2 "xdotool が見つかりません。"; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo >&2 "ros2 が見つかりません。ROS 2環境をsourceしてください。"; exit 1; }
command -v wmctrl >/dev/null 2>&1 || { echo >&2 "wmctrl が見つかりません。sudo apt install wmctrl を実行してください。"; exit 1; }


# --- 3. 録画時間 (Duration) を ros2 bag info から自動取得 ---
echo "ros2 bag info から録画時間を読み込み中..."
BAG_INFO=$(ros2 bag info "$MCAP_FILE" 2>/dev/null)

if ! echo "$BAG_INFO" | grep -q "Duration:"; then
    echo "エラー: 'ros2 bag info' の実行に失敗しました。"
    ros2 bag info "$MCAP_FILE"
    exit 1
fi

DURATION_SEC=$(echo "$BAG_INFO" | awk '/Duration:/ {print $2}' | tr -d 's')

# 小数点以下切り上げ (ceil)
DURATION=$(echo $DURATION_SEC | awk '{ val = $1; print int(val) + (val > int(val)) }')

if [ $DURATION -le 0 ]; then
    echo "エラー: rosbagのdurationが0秒以下です。"
    exit 1
fi

# ★ 録画マージンとして3秒追加
DURATION=$((DURATION + 3))

# --- 出力ファイル名の設定 ---
OUTPUT_FILE="${MCAP_FILE%.mcap}.mp4"
if [ "$MCAP_FILE" == "$OUTPUT_FILE" ]; then
    echo "エラー: 入力ファイル名が .mcap で終わっていません。"
    exit 1
fi

echo "入力ファイル: $MCAP_FILE"
echo "出力ファイル: $OUTPUT_FILE"
echo "録画時間: ${DURATION}秒 (ros2 bag info + 5秒マージン)"


# --- 4. lichtblick の起動 ---
echo "lichtblick を起動します: $MCAP_FILE"
lichtblick "$MCAP_FILE" &
LICHTBLICK_PID=$!

# --- 5. ウィンドウの特定 ---
echo "lichtblick のメインウィンドウの起動を待機中 (5秒)..."
sleep 5

WINDOW_IDS=$(xdotool search --classname "$TARGET_CLASSNAME")

if [ -z "$WINDOW_IDS" ]; then
    echo "エラー: lichtblick のウィンドウ (Class: $TARGET_CLASSNAME) が見つかりませんでした。"
    kill $LICHTBLICK_PID
    exit 1
fi

MAX_AREA=0
WINDOW_ID=""

echo "検出された '$TARGET_CLASSNAME' ウィンドウをスキャン中..."
for id in $WINDOW_IDS; do
    GEOMETRY=$(xdotool getwindowgeometry $id 2>/dev/null)
    if [ -z "$GEOMETRY" ]; then
      continue
    fi

    SIZE_WH=$(echo "$GEOMETRY" | grep "Geometry" | awk '{print $2}')
    W=$(echo $SIZE_WH | cut -dx -f1)
    H=$(echo $SIZE_WH | cut -dx -f2)
    if [ -z "$W" ] || [ -z "$H" ] || [ "$W" -eq 0 ] || [ "$H" -eq 0 ]; then
      continue
    fi
    AREA=$((W * H))

    echo "  -> ID: $id, サイズ: ${W}x${H}, 面積: $AREA"

    if [ $AREA -gt $MAX_AREA ]; then
        MAX_AREA=$AREA
        WINDOW_ID=$id
    fi
done

if [ -z "$WINDOW_ID" ] || [ $MAX_AREA -lt 10000 ]; then
    echo "エラー: 有効なメインウィンドウが見つかりませんでした。"
    kill $LICHTBLICK_PID
    exit 1
fi

echo "メインウィンドウを発見しました (ID: $WINDOW_ID, 面積: $MAX_AREA)"

# --- 6. ウィンドウの最大化と座標取得 ---
echo "ウィンドウを最大化します (wmctrl)..."
# ★ xdotool windowmaximize の代わりに wmctrl を使用
# '-i' で10進数のIDを、'-r' でIDを指定し、'-b add' で状態を追加
wmctrl -i -r $WINDOW_ID -b add,maximized_vert,maximized_horz

# 最大化のアニメーション/リサイズが完了するのを待つ (少し長めに)
sleep 1.5

# 最大化後の座標とサイズを再取得
GEOMETRY=$(xdotool getwindowgeometry $WINDOW_ID)
POS_XY=$(echo "$GEOMETRY" | grep "Position" | awk '{print $2}')
SIZE_WH=$(echo "$GEOMETRY" | grep "Geometry" | awk '{print $2}')
echo "録画範囲 (最大化): サイズ=$SIZE_WH, 座標=$POS_XY"

# --- 7. 再生の開始 ---
echo "ファイル読み込み待ち (3秒)..."
sleep 3
echo "ウィンドウをフォーカスして再生します (キー送信)..."
xdotool windowactivate $WINDOW_ID
sleep 0.2
xdotool key --window $WINDOW_ID space
sleep 0.5

# --- 8. ffmpeg による録画実行 ---
echo "録画を開始します ($DURATION 秒間)..."

ffmpeg -f x11grab -draw_mouse 0 -r 30 -s $SIZE_WH -i :0.0+$POS_XY -t $DURATION \
       -vf "crop=floor(iw/2)*2:floor(ih/2)*2" \
       -loglevel warning -y "$OUTPUT_FILE" &
# 低品質・少サイズ版
# ffmpeg -f x11grab -draw_mouse 0 -r 24 -s $SIZE_WH -i :0.0+$POS_XY -t $DURATION \
#        -vf "scale=1280:-1,crop=floor(iw/2)*2:floor(ih/2)*2" \
#        -c:v libvpx-vp9 -b:v 500k -speed 4 -an \
#        -loglevel warning -y "$OUTPUT_FILE" &

FFMPEG_PID=$!

wait $FFMPEG_PID

# --- 9. クリーンアップ ---
echo "録画が完了しました: $OUTPUT_FILE"
echo "lichtblick (PID: $LICHTBLICK_PID) を終了します。"
kill $LICHTBLICK_PID

echo "スクリプト完了。"