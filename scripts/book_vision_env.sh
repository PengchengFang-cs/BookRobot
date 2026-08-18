#!/usr/bin/env bash

export PYTHONPATH="/home/unix_ai/.local/share/bookbot/vision-rpc/runtime/sysroot/usr/lib/python3/dist-packages:/home/unix_ai/.local/share/bookbot/vision-rpc/runtime/client${PYTHONPATH:+:$PYTHONPATH}"
export BOOK_VISION_ENDPOINT=127.0.0.1:7443
export BOOK_VISION_SERVER_NAME=planning-server.bookbot.internal
export BOOK_VISION_PROTO_DIR=/home/unix_ai/.local/share/bookbot/vision-rpc/runtime/client/vision_generated
export BOOK_VISION_CA=/home/unix_ai/.local/share/bookbot/vision-rpc/certs/wanda-vision-rpc-ca-20260815.pem
export BOOK_VISION_CERT=/home/unix_ai/.local/share/bookbot/vision-rpc/certs/wanda-vision-client-20260815.pem
export BOOK_VISION_KEY=/home/unix_ai/.local/share/bookbot/vision-rpc/private/wanda-vision-client.key
export BOOK_VISION_SOURCE=ruan/visiond
export BOOK_VISION_WORKER_ID=onsite-gpu0-vision
export BOOK_VISION_MODEL_VERSION=grounding-dino-base-swinb+sam2.1-hiera-large+ppocrv6-20260814-r3
export BOOK_VISION_CONFIG_HASH=bb6af3d199e3c41051a4844ae122ebf1e0a7bd63496f33f22f78e9ee0599371f
export BOOK_VISION_CALIBRATION_VERSION=wanda-head-rgbd-live-20260815
