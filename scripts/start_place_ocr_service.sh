#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$ROOT/logs"
PYTHON=/home/cvailab/Ruan/WHRCompetition/runtime/book-vision-unified-20260814-r4/env/bin/python
GPU_UUID=GPU-73a33248-6284-4b58-4f4f-b37e711fd4ae
SERVER_UNIT=fpc-place-ocr-gpu7.service
TUNNEL_UNIT=fpc-place-ocr-tunnel.service

mkdir -p "$LOG_DIR"

if systemctl --user is-active --quiet "$SERVER_UNIT"; then
  printf 'GPU 7 Place OCR服务已运行\n'
else
  systemctl --user reset-failed "$SERVER_UNIT" 2>/dev/null || true
  systemd-run --user \
    --unit="$SERVER_UNIT" \
    --collect \
    --property=Restart=on-failure \
    --property=RestartSec=2s \
    --property="StandardOutput=append:$LOG_DIR/place_ocr_gpu7.log" \
    --property="StandardError=append:$LOG_DIR/place_ocr_gpu7.log" \
    --setenv="CUDA_VISIBLE_DEVICES=$GPU_UUID" \
    "$PYTHON" "$ROOT/scripts/place_ocr_server.py"
  printf '已启动GPU 7 Place OCR服务\n'
fi

if systemctl --user is-active --quiet "$TUNNEL_UNIT"; then
  printf '机器人7445转发已运行\n'
else
  systemctl --user reset-failed "$TUNNEL_UNIT" 2>/dev/null || true
  systemd-run --user \
    --unit="$TUNNEL_UNIT" \
    --collect \
    --property=Restart=on-failure \
    --property=RestartSec=2s \
    --property="StandardOutput=append:$LOG_DIR/place_ocr_tunnel.log" \
    --property="StandardError=append:$LOG_DIR/place_ocr_tunnel.log" \
    /usr/bin/ssh -NT \
    -o ExitOnForwardFailure=yes \
    -o BatchMode=yes \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 \
    -p 22024 \
    -R 127.0.0.1:7445:127.0.0.1:7445 \
    unix_ai@127.0.0.1
  printf '已启动机器人7445转发\n'
fi
