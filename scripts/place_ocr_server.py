#!/usr/bin/env python3
"""GPU-isolated SAM + raw PP-OCRv6 service for Stage-1 Place frames."""

from __future__ import annotations

import base64
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path
import re
import sys
import time


PROJECT_ROOT = Path(
    "/home/cvailab/Ruan/WHRCompetition/runtime/"
    "bookbot-visiond-competition-20260815-r1/project"
)
HOST_PROFILE = PROJECT_ROOT.parent / "runtime/onsite_single_5090.competition-provisional.json"
LEGACY_CONFIG = PROJECT_ROOT / "configs/legacy_traditional_vision.competition.runtime.json"
VIEWPOINT_LAYOUT = PROJECT_ROOT.parent / "runtime/inspection_viewpoint_layout.example.json"
INVENTORY_CATALOG = PROJECT_ROOT.parent / "runtime/inspection_inventory.example.json"
LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 7445
MAXIMUM_REQUEST_BYTES = 64 * 1024 * 1024
EXPECTED_OFFSETS = tuple(range(-12, -4))
BOOK_LABEL_RE = re.compile(r"^A[0-9]{2}-[0-9]{4}$")
PREFIX_RE = re.compile(r"^(A[0-9]{2})-?$")
SERIAL_RE = re.compile(r"^[0-9]{4}$")


sys.path.insert(0, str(PROJECT_ROOT / "src"))


class PlaceOcrRuntime:
    def __init__(self):
        from bookbot_vision.shelf_ocr_server import build_ready_router

        self.router = build_ready_router(
            host_profile=HOST_PROFILE,
            viewpoint_layout=VIEWPOINT_LAYOUT,
            inventory_catalog=INVENTORY_CATALOG,
            legacy_config=LEGACY_CONFIG,
            project_root=PROJECT_ROOT,
        )
        self.worker = self.router.shared_book_worker
        self.model = self.worker.inference_adapter.model
        self.ocr = self.worker.inference_adapter.label_reader.backend

    @staticmethod
    def _decode_jpeg(payload):
        import cv2
        import numpy as np

        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise RuntimeError("Place OCR JPEG解码失败")
        return image

    @staticmethod
    def _bbox_from_mask(mask, image_shape):
        import numpy as np

        value = mask
        for method_name in ("detach", "cpu"):
            method = getattr(value, method_name, None)
            if callable(method):
                value = method()
        to_numpy = getattr(value, "numpy", None)
        if callable(to_numpy):
            value = to_numpy()
        array = np.asarray(value)
        array = np.squeeze(array)
        if array.shape != tuple(image_shape[:2]):
            raise RuntimeError("SAM mask尺寸与图像不一致")
        rows, columns = np.nonzero(array.astype(bool))
        if rows.size == 0:
            raise RuntimeError("SAM返回空书本mask")
        x0, x1 = int(columns.min()), int(columns.max())
        y0, y1 = int(rows.min()), int(rows.max())
        return x0, y0, x1 + 1, y1 + 1

    @staticmethod
    def _raw_lines(output):
        texts = tuple(getattr(output, "txts", ()) or ())
        scores = tuple(getattr(output, "scores", ()) or ())
        if len(texts) != len(scores):
            raise RuntimeError("PP-OCRv6文本与置信度数量不一致")
        return tuple(
            (str(text), float(score))
            for text, score in zip(texts, scores)
        )

    @staticmethod
    def _label_from_lines(lines):
        normalized = tuple(
            ("".join(text.upper().split()), confidence)
            for text, confidence in lines
        )
        candidates = []
        for text, confidence in normalized:
            if BOOK_LABEL_RE.fullmatch(text) is not None:
                candidates.append((text, confidence))
        prefixes = []
        serials = []
        for text, confidence in normalized:
            prefix = PREFIX_RE.fullmatch(text)
            if prefix is not None:
                prefixes.append((prefix.group(1), confidence))
            if SERIAL_RE.fullmatch(text) is not None:
                serials.append((text, confidence))
        for prefix, prefix_confidence in prefixes:
            for serial, serial_confidence in serials:
                candidates.append((
                    f"{prefix}-{serial}",
                    min(prefix_confidence, serial_confidence),
                ))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[1])

    def _infer_frame(self, row):
        from bookbot_vision.book_instance.model_adapter import BookInstanceFrame

        payload = base64.b64decode(row["jpeg_base64"], validate=True)
        image = self._decode_jpeg(payload)
        captured_at_ns = int(row["captured_at_ns"])
        offset = int(row["release_offset"])
        digest = hashlib.sha256(payload).hexdigest()
        frame = BookInstanceFrame(
            capture_id=f"place-{captured_at_ns}",
            camera_id="head_rgbd",
            captured_at_ns=captured_at_ns,
            width_px=int(image.shape[1]),
            height_px=int(image.shape[0]),
            encoding="jpeg",
            payload=payload,
            frame_sha256=digest,
            base_motion_epoch=f"place-base-{captured_at_ns}",
            head_motion_epoch=f"place-head-{captured_at_ns}",
        )
        candidates = tuple(
            candidate
            for candidate in self.model.predict(frame)
            if candidate.class_name == "book"
        )
        if not candidates:
            raise RuntimeError("SAM未检测到书本")
        located = tuple(
            (self._bbox_from_mask(candidate.mask, image.shape), candidate)
            for candidate in candidates
        )
        bbox, selected = max(
            located,
            key=lambda item: (item[0][0] + item[0][2]) / 2.0,
        )
        x0, y0, x1, y1 = bbox
        crop = image[y0:y1, x0:x1]
        lines = self._raw_lines(self.ocr.infer_text_lines(crop))
        label = self._label_from_lines(lines)
        result = {
            "release_offset": offset,
            "sam_book_count": len(candidates),
            "selected_bbox_xywh": [x0, y0, x1 - x0, y1 - y0],
            "sam_confidence": float(selected.confidence),
            "ocr_lines": [
                {"text": text, "confidence": confidence}
                for text, confidence in lines
            ],
            "label": None if label is None else label[0],
            "confidence": None if label is None else label[1],
        }
        return result

    def infer(self, payload):
        rows = payload.get("frames")
        if not isinstance(rows, list) or len(rows) != len(EXPECTED_OFFSETS):
            raise RuntimeError("Place OCR请求必须包含8张图像")
        ordered = sorted(rows, key=lambda row: int(row["release_offset"]))
        if tuple(int(row["release_offset"]) for row in ordered) != EXPECTED_OFFSETS:
            raise RuntimeError("Place OCR图像必须覆盖-12到-5")
        results = []
        with self.worker.external_inference_lane():
            for row in ordered:
                try:
                    results.append(self._infer_frame(row))
                except Exception as error:
                    results.append({
                        "release_offset": int(row["release_offset"]),
                        "label": None,
                        "confidence": None,
                        "error": type(error).__name__,
                    })
        successful = tuple(row for row in results if row.get("label") is not None)
        if not successful:
            return {"label": None, "confidence": None, "frames": results}
        selected = max(successful, key=lambda row: float(row["confidence"]))
        return {
            "label": selected["label"],
            "confidence": selected["confidence"],
            "selected_release_offset": selected["release_offset"],
            "frames": results,
        }

    def close(self):
        self.router.close()


class PlaceOcrHandler(BaseHTTPRequestHandler):
    runtime = None

    def _send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != "/health":
            self._send_json(404, {"error": "not_found"})
            return
        self._send_json(200, {"ready": True, "gpu": 7})

    def do_POST(self):
        if self.path != "/infer":
            self._send_json(404, {"error": "not_found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 1 <= length <= MAXIMUM_REQUEST_BYTES:
                raise RuntimeError("Place OCR请求大小无效")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            self._send_json(200, self.runtime.infer(payload))
        except Exception as error:
            print(f"PLACE_OCR_REQUEST_ERROR {type(error).__name__}", flush=True)
            self._send_json(500, {"error": type(error).__name__})

    def log_message(self, format, *args):
        print(
            "%s - - [%s] %s"
            % (self.address_string(), self.log_date_time_string(), format % args),
            flush=True,
        )


def main():
    runtime = PlaceOcrRuntime()
    PlaceOcrHandler.runtime = runtime
    server = HTTPServer((LISTEN_HOST, LISTEN_PORT), PlaceOcrHandler)
    print(
        f"PLACE_OCR_SERVICE_READY endpoint={LISTEN_HOST}:{LISTEN_PORT} "
        f"started_at_ns={time.time_ns()}",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
