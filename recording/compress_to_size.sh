#!/usr/bin/env bash
# Compresses a video to (just under) a target file size with two-pass H.264, keeping the
# resolution. This is how the Flatbed Falcon final cut went from 83 MB to 9.2 MB
# (1920x1080, 2:55, 25 fps -> about 440 kbit/s video, preset slow, no audio). Text, plots
# and labels stay sharp; fast camera motion over trees gets a little soft.
#
#   recording/video.sh compress in.mp4 out_10mb.mp4            # target 10 MB
#   recording/video.sh compress in.mp4 out.mp4 25              # target 25 MB
#   recording/video.sh compress in.mp4 out.mp4 10 1280         # also scale to 1280 px wide
#
# Needs ffmpeg + ffprobe. Keeps audio at 96 kbit/s if the input has any.
set -e
IN="$1"; OUT="$2"; MB="${3:-10}"; WIDTH="${4:-}"
[ -f "$IN" ] && [ -n "$OUT" ] || { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$IN")
HAS_AUDIO=$(ffprobe -v error -select_streams a -show_entries stream=index -of csv=p=0 "$IN" | head -n1)
AUDIO_K=0; AUDIO_ARGS=(-an)
[ -n "$HAS_AUDIO" ] && { AUDIO_K=96; AUDIO_ARGS=(-c:a aac -b:a 96k); }
# MB = 10^6 bytes. 95 % of the budget, to leave room for the mp4 container overhead.
VIDEO_K=$(python3 -c "print(max(50, int($MB * 8000 * 0.95 / $DUR - $AUDIO_K)))")
SCALE=(); [ -n "$WIDTH" ] && SCALE=(-vf "scale=${WIDTH}:-2")
echo "$(basename "$IN"): ${DUR%.*} s -> video ${VIDEO_K} kbit/s for about ${MB} MB"

PASSLOG=$(mktemp -u /tmp/x264pass.XXXX)
ffmpeg -v error -stats -y -i "$IN" "${SCALE[@]}" -c:v libx264 -preset slow -b:v "${VIDEO_K}k" \
  -pass 1 -passlogfile "$PASSLOG" -pix_fmt yuv420p -an -f mp4 /dev/null
ffmpeg -v error -stats -y -i "$IN" "${SCALE[@]}" -c:v libx264 -preset slow -b:v "${VIDEO_K}k" \
  -pass 2 -passlogfile "$PASSLOG" -pix_fmt yuv420p "${AUDIO_ARGS[@]}" -movflags +faststart "$OUT"
rm -f "$PASSLOG"*
echo "Saved $OUT ($(python3 -c "import os; print(f'{os.path.getsize(\"$OUT\") / 1e6:.1f}')") MB)"
