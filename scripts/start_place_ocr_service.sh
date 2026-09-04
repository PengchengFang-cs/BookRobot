#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$ROOT/logs"
PYTHON=/home/cvailab/Ruan/WHRCompetition/runtime/book-vision-unified-20260814-r4/env/bin/python
GPU_UUID=GPU-73a33248-6284-4b58-4f4f-b37e711fd4ae
SERVER_PID_FILE="$LOG_DIR/place_ocr_gpu7.pid"
TUNNEL_PID_FILE="$LOG_DIR/place_ocr_tunnel.pid"

mkdir -p "$LOG_DIR"

if [[ -s "$SERVER_PID_FILE" ]] && kill -0 "$(<"$SERVER_PID_FILE")" 2>/dev/null; then
  printf 'GPU 7 Place OCR服务已运行: pid=%s\n' "$(<"$SERVER_PID_FILE")"
else
  CUDA_VISIBLE_DEVICES="$GPU_UUID" \
    nohup "$PYTHON" "$ROOT/scripts/place_ocr_server.py" \
    >"$LOG_DIR/place_ocr_gpu7.log" 2>&1 &
  printf '%s\n' "$!" >"$SERVER_PID_FILE"
  printf '已启动GPU 7 Place OCR服务: pid=%s\n' "$!"
fi

if [[ -s "$TUNNEL_PID_FILE" ]] && kill -0 "$(<"$TUNNEL_PID_FILE")" 2>/dev/null; then
  printf '机器人7445转发已运行: pid=%s\n' "$(<"$TUNNEL_PID_FILE")"
else
  nohup /usr/bin/ssh -NT \
    -o ExitOnForwardFailure=yes \
    -o BatchMode=yes \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 \
    -p 22024 \
    -R 127.0.0.1:7445:127.0.0.1:7445 \
    unix_ai@127.0.0.1 \
    >"$LOG_DIR/place_ocr_tunnel.log" 2>&1 &
  printf '%s\n' "$!" >"$TUNNEL_PID_FILE"
  printf '已启动机器人7445转发: pid=%s\n' "$!"
fi
