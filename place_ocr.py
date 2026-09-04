"""Background Stage-1 Place OCR and the Stage-2 JSON handoff."""

from concurrent.futures import ThreadPoolExecutor
import base64
import json
import os
from pathlib import Path
import re
from urllib.request import Request, urlopen

from config import PLACE_OCR_ENDPOINT, STAGE1_CART_BOOKS_PATH


BOOK_LABEL_RE = re.compile(r"^A[0-9]{2}-[0-9]{4}$")
PLACE_OCR_TIMEOUT_S = 120.0


class Stage1PlaceOcrRecorder:
    def __init__(self, *, reset):
        self.output_path = Path(STAGE1_CART_BOOKS_PATH)
        self.run_id = os.environ.get("FPC_EXPERIMENT_RUN_ID", "unknown")
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.futures = []
        self.books = [] if reset else self._load_existing_books()
        if reset:
            self._write_json()

    def _load_existing_books(self):
        if not self.output_path.is_file():
            return []
        payload = json.loads(self.output_path.read_text(encoding="utf-8"))
        rows = payload.get("books_left_to_right")
        if not isinstance(rows, list):
            raise RuntimeError("Stage 1 书本OCR JSON格式无效")
        return rows

    def _write_json(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "updated_run_id": self.run_id,
            "books_left_to_right": sorted(
                self.books,
                key=lambda row: int(row["place_index"]),
            ),
        }
        self.output_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _request(captures):
        frames = []
        for capture in captures:
            path = Path(capture.path)
            frames.append({
                "release_offset": int(capture.release_offset),
                "captured_at_ns": int(capture.captured_at_ns),
                "path": str(path),
                "jpeg_base64": base64.b64encode(path.read_bytes()).decode("ascii"),
            })
        body = json.dumps({"frames": frames}).encode("utf-8")
        request = Request(
            PLACE_OCR_ENDPOINT,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=PLACE_OCR_TIMEOUT_S) as response:
            result = json.loads(response.read().decode("utf-8"))
        label = result.get("label")
        if not isinstance(label, str) or BOOK_LABEL_RE.fullmatch(label) is None:
            raise RuntimeError("8张Place图像均未识别出完整书本标签")
        return result

    def _process(self, book_index, captures):
        result = self._request(captures)
        row = {
            "place_index": int(book_index),
            "label": result["label"],
            "confidence": float(result["confidence"]),
            "selected_release_offset": int(result["selected_release_offset"]),
            "source_run_id": self.run_id,
            "frames": result.get("frames", []),
        }
        self.books = [
            existing
            for existing in self.books
            if int(existing["place_index"]) != int(book_index)
        ]
        self.books.append(row)
        self._write_json()
        print(
            "PLACE_OCR "
            f"book_index={book_index} label={row['label']} "
            f"confidence={row['confidence']:.6f} "
            f"release_offset={row['selected_release_offset']:+d} "
            f"json={self.output_path}",
            flush=True,
        )

    def submit(self, book_index, captures):
        if type(book_index) is not int or book_index < 1:
            raise RuntimeError("Place OCR书本顺序无效")
        frames = tuple(captures)
        if len(frames) != 8:
            raise RuntimeError(
                f"Place OCR需要8张图像，实际收到{len(frames)}张"
            )
        self.futures.append(
            self.executor.submit(self._process, book_index, frames)
        )

    def close(self):
        self.executor.shutdown(wait=True)
        for future in self.futures:
            future.result()


def load_stage1_cart_book_labels(path=STAGE1_CART_BOOKS_PATH):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = payload.get("books_left_to_right")
    if not isinstance(rows, list) or len(rows) != 2:
        raise RuntimeError("Stage 2需要Stage 1从左到右记录的两本书")
    ordered = sorted(rows, key=lambda row: int(row["place_index"]))
    if [int(row["place_index"]) for row in ordered] != [1, 2]:
        raise RuntimeError("Stage 1书本顺序必须是1、2")
    labels = tuple(str(row.get("label", "")) for row in ordered)
    if any(BOOK_LABEL_RE.fullmatch(label) is None for label in labels):
        raise RuntimeError("Stage 1书本OCR JSON缺少完整标签")
    return labels
