#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOCAL_DIR="$ROOT/logs/record_rebot/$RUN_ID"
REMOTE_DIR="/home/unix_ai/fpc/logs/record_rebot/$RUN_ID"
mkdir -p "$LOCAL_DIR"

REMOTE_COMMAND="cd /home/unix_ai/fpc && FPC_EXPERIMENT_RUN_ID=$RUN_ID ./run.sh"
for argument in "$@"; do
  printf -v quoted_argument '%q' "$argument"
  REMOTE_COMMAND+=" $quoted_argument"
done

copy_record() {
  scp -q -r "tsinghuaBot:$REMOTE_DIR/." "$LOCAL_DIR/" || \
    printf '实验记录传回失败，机器人端仍保留在 %s\n' "$REMOTE_DIR" >&2
}

trap copy_record EXIT
ssh -tt tsinghuaBot "$REMOTE_COMMAND"
STATUS=$?
trap - EXIT
copy_record
printf '实验记录: %s\n' "$LOCAL_DIR"
exit "$STATUS"
