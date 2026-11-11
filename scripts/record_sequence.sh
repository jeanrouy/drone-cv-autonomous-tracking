#!/usr/bin/env bash
# Enregistre une sequence video de reference sans recompression :
# le flux MJPEG de la camera est copie tel quel (cout CPU quasi nul, horodatage conserve).
# Usage : ./scripts/record_sequence.sh data/sequences/ref_01.mkv [duree_s]
set -e

OUTPUT="$1"
DURATION="${2:-60}"
CAMERA="${FM_CAMERA:?Variable FM_CAMERA non definie (voir ~/.bashrc)}"

if [ -z "$OUTPUT" ]; then
  echo "Usage : $0 <fichier.mkv> [duree_s]"
  exit 1
fi

# Interdit a la camera de diviser sa cadence par deux en faible lumiere
v4l2-ctl -d "$CAMERA" --set-ctrl=exposure_dynamic_framerate=0

mkdir -p "$(dirname "$OUTPUT")"
for i in 3 2 1; do
  echo "Debut dans $i..."
  sleep 1
done

ffmpeg -hide_banner -loglevel warning -stats \
  -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 30 -i "$CAMERA" \
  -t "$DURATION" -c:v copy "$OUTPUT"

echo "Verification du fichier :"
ffprobe -v error -count_frames -select_streams v:0 \
  -show_entries stream=width,height,avg_frame_rate,nb_read_frames \
  -of default=noprint_wrappers=1 "$OUTPUT"