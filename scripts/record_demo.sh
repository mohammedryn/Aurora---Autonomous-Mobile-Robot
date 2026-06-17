#!/bin/bash
# Record the full X display to an MP4 demo video.
# Run this in a second terminal AFTER launching demo_sim.sh.
# Press Ctrl+C to stop recording.
set -e

OUTPUT="aurora_demo_$(date +%Y%m%d_%H%M%S).mp4"
RESOLUTION=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")

echo "======================================"
echo "  Aurora Demo Recorder"
echo "======================================"
echo "  Output : ${OUTPUT}"
echo "  Display: ${DISPLAY}"
echo "  Res    : ${RESOLUTION}"
echo "  Press Ctrl+C to stop."
echo "======================================"

ffmpeg \
  -f x11grab \
  -r 30 \
  -s "${RESOLUTION}" \
  -i "${DISPLAY}" \
  -c:v libx264 \
  -preset fast \
  -crf 18 \
  -pix_fmt yuv420p \
  "${OUTPUT}"

echo "Saved to: ${OUTPUT}"
