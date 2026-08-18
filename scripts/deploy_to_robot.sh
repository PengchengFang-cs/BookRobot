#!/usr/bin/env bash
set -euo pipefail

repo_root=/home/cvailab/fpc
robot_root=/home/unix_ai/fpc

cd "$repo_root"

if [[ $(git branch --show-current) != main ]]; then
    echo 'error: deployment is allowed only from main' >&2
    exit 2
fi
if [[ -n $(git status --porcelain) ]]; then
    echo 'error: commit local changes before deployment' >&2
    exit 2
fi

local_sha=$(git rev-parse HEAD)
export_dir=$(mktemp -d /tmp/bookrobot-deploy.XXXXXX)
trap 'rm -rf -- "$export_dir"' EXIT

git archive HEAD | tar -x -C "$export_dir"
ssh tsinghuaBot "mkdir -p '$robot_root'"
rsync --archive "$export_dir/" "tsinghuaBot:$robot_root/"
printf '%s\n' "$local_sha" | ssh tsinghuaBot \
    "read -r deployed_sha && printf '%s\\n' \"\$deployed_sha\" > '$robot_root/.deployed-commit'"

printf 'deployed=%s\n' "$local_sha"
