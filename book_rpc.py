"""Small client for the fixed-profile 5090 table-book mask service."""

from dataclasses import dataclass
import hashlib
import importlib.util
from pathlib import Path
import time
import uuid

import numpy as np

from book_geometry import BookMask


SCENE_TASK = "scene_table_books_segmentation"
SCENE_PROFILE = "table_books_v1"
CART_SCENE_TASK = "scene_cart_loading_segmentation"
CART_SCENE_PROFILE = "cart_loading_v1"
CART_SCENE_CLASSES = ("cart_body", "cart_platform", "book")
VISION_METHOD = "/bookbot.vision.v2.VisionService/Infer"
MAX_REQUEST_BYTES = 64 * 1024 * 1024
MAX_RESPONSE_BYTES = 16 * 1024 * 1024


class BookVisionError(RuntimeError):
    pass


@dataclass(frozen=True)
class SceneMask:
    semantic_class: str
    scene_profile_id: str
    confidence: float
    bbox: tuple[int, int, int, int]
    rle_counts: tuple[int, ...]
    image_width: int
    image_height: int


def grpc_channel_options(server_name):
    return (
        ("grpc.ssl_target_name_override", server_name),
        ("grpc.default_authority", server_name),
        ("grpc.max_send_message_length", MAX_REQUEST_BYTES),
        ("grpc.max_receive_message_length", MAX_RESPONSE_BYTES),
    )


@dataclass(frozen=True)
class BookVisionSettings:
    endpoint: str
    server_name: str
    ca_certificate_path: str
    client_certificate_path: str
    client_private_key_path: str
    expected_source: str
    expected_worker_id: str
    expected_model_version: str
    config_hash: str
    calibration_version: str
    protobuf_directory: str = ""
    timeout_s: float = 3.0

    def validate(self):
        required = (
            self.endpoint,
            self.server_name,
            self.expected_source,
            self.expected_worker_id,
            self.expected_model_version,
            self.config_hash,
            self.calibration_version,
        )
        if any(not isinstance(value, str) or not value.strip() for value in required):
            raise BookVisionError("book_vision_configuration_incomplete")
        if (
            not np.isfinite(self.timeout_s)
            or self.timeout_s <= 0
            or self.timeout_s > 2.5
        ):
            raise BookVisionError("book_vision_timeout_invalid")


def _load_pb2(directory):
    path = Path(directory) / "vision_v2_pb2.py"
    if not path.is_file():
        raise BookVisionError(f"vision_protobuf_not_found: {path}")
    spec = importlib.util.spec_from_file_location("fruittest_vision_v2_pb2", path)
    if spec is None or spec.loader is None:
        raise BookVisionError("vision_protobuf_load_failed")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise BookVisionError("vision_protobuf_load_failed") from error
    return module


def _read_bytes(path, label):
    try:
        value = Path(path).read_bytes()
    except OSError as error:
        raise BookVisionError(f"{label}_unreadable: {path}") from error
    if not value:
        raise BookVisionError(f"{label}_empty")
    return value


class BookVisionClient:
    def __init__(self, settings, *, pb2_module=None, rpc=None, clock_ns=time.time_ns):
        if not isinstance(settings, BookVisionSettings):
            raise TypeError("settings must be BookVisionSettings")
        settings.validate()
        self.settings = settings
        self.pb2 = pb2_module or _load_pb2(settings.protobuf_directory)
        if not callable(clock_ns):
            raise TypeError("clock_ns must be callable")
        self._clock_ns = clock_ns
        self._channel = None
        self._rpc = rpc or self._build_rpc()
        self._sequence = 0

    @classmethod
    def from_config(cls):
        from config import (
            BOOK_VISION_CA,
            BOOK_VISION_CALIBRATION_VERSION,
            BOOK_VISION_CERT,
            BOOK_VISION_CONFIG_HASH,
            BOOK_VISION_ENDPOINT,
            BOOK_VISION_KEY,
            BOOK_VISION_MODEL_VERSION,
            BOOK_VISION_PROTO_DIR,
            BOOK_VISION_SERVER_NAME,
            BOOK_VISION_SOURCE,
            BOOK_VISION_TIMEOUT_S,
            BOOK_VISION_WORKER_ID,
        )

        return cls(
            BookVisionSettings(
                endpoint=BOOK_VISION_ENDPOINT,
                server_name=BOOK_VISION_SERVER_NAME,
                ca_certificate_path=BOOK_VISION_CA,
                client_certificate_path=BOOK_VISION_CERT,
                client_private_key_path=BOOK_VISION_KEY,
                expected_source=BOOK_VISION_SOURCE,
                expected_worker_id=BOOK_VISION_WORKER_ID,
                expected_model_version=BOOK_VISION_MODEL_VERSION,
                config_hash=BOOK_VISION_CONFIG_HASH,
                calibration_version=BOOK_VISION_CALIBRATION_VERSION,
                protobuf_directory=BOOK_VISION_PROTO_DIR,
                timeout_s=BOOK_VISION_TIMEOUT_S,
            )
        )

    def _build_rpc(self):
        try:
            import grpc
        except ImportError as error:
            raise BookVisionError("grpc_not_installed") from error

        credentials = grpc.ssl_channel_credentials(
            root_certificates=_read_bytes(
                self.settings.ca_certificate_path, "vision_ca"
            ),
            private_key=_read_bytes(
                self.settings.client_private_key_path, "vision_client_key"
            ),
            certificate_chain=_read_bytes(
                self.settings.client_certificate_path, "vision_client_certificate"
            ),
        )
        self._channel = grpc.secure_channel(
            self.settings.endpoint,
            credentials,
            options=grpc_channel_options(self.settings.server_name),
        )
        return self._channel.unary_unary(
            VISION_METHOD,
            request_serializer=lambda message: message.SerializeToString(
                deterministic=True
            ),
            response_deserializer=self.pb2.InferResponse.FromString,
        )

    def _make_request(
        self,
        image_bgr,
        *,
        captured_at_ns,
        base_motion_epoch,
        head_motion_epoch,
        task=SCENE_TASK,
    ):
        image = np.asarray(image_bgr)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise BookVisionError("book_vision_image_must_be_bgr8")
        image = np.ascontiguousarray(image)
        if type(captured_at_ns) is not int or captured_at_ns <= 0:
            raise BookVisionError("book_vision_capture_time_invalid")
        if not base_motion_epoch or not head_motion_epoch:
            raise BookVisionError("book_vision_motion_epoch_missing")

        self._sequence += 1
        now_ns = self._clock_ns()
        lifetime_ns = int(self.settings.timeout_s * 1_000_000_000)
        deadline_ns = captured_at_ns + lifetime_ns
        if now_ns >= deadline_ns:
            raise BookVisionError("book_vision_capture_expired")
        request_id = f"fruittest-{uuid.uuid4()}"
        capture_id = f"head-{captured_at_ns}"
        request = self.pb2.InferRequest()
        header = request.header
        header.schema_version = 2
        header.mission_run_id = "fruittest-book-perception"
        header.stage_id = "perception-test"
        header.request_id = request_id
        header.correlation_id = request_id
        header.sequence = self._sequence
        # The reviewed service requires issued <= not_before <= capture <= now.
        # This request is born from one already-captured image, so its lifetime
        # starts at that capture rather than at the later RPC construction time.
        header.issued_at_ns = captured_at_ns
        header.not_before_ns = captured_at_ns
        header.deadline_ns = deadline_ns
        header.expected_output_frame = "image_pixels"
        header.config_hash = self.settings.config_hash
        header.calibration_version = self.settings.calibration_version

        request.task = str(task)
        request.expected_source = self.settings.expected_source
        request.expected_worker_id = self.settings.expected_worker_id
        request.expected_model_version = self.settings.expected_model_version
        capture = request.captures.add()
        capture.capture_id = capture_id
        capture.camera_id = "head_rgbd"
        capture.base_motion_epoch = str(base_motion_epoch)
        capture.head_motion_epoch = str(head_motion_epoch)
        capture.sequence_index = 0
        color = capture.color
        color.camera_id = "head_rgbd"
        color.optical_frame_id = "head_rgbd_color_optical_frame"
        color.captured_at_ns = captured_at_ns
        color.encoding = "bgr8"
        color.width = int(image.shape[1])
        color.height = int(image.shape[0])
        color.payload = image.tobytes()
        color.payload_sha256 = hashlib.sha256(color.payload).hexdigest()
        return request

    def detect(
        self,
        image_bgr,
        *,
        captured_at_ns,
        base_motion_epoch,
        head_motion_epoch,
    ):
        rows = self.detect_scene(
            image_bgr,
            captured_at_ns=captured_at_ns,
            base_motion_epoch=base_motion_epoch,
            head_motion_epoch=head_motion_epoch,
            task=SCENE_TASK,
            profile=SCENE_PROFILE,
            semantic_classes=("book",),
        )
        return tuple(
            BookMask(
                confidence=row.confidence,
                bbox=row.bbox,
                rle_counts=row.rle_counts,
                image_width=row.image_width,
                image_height=row.image_height,
            )
            for row in rows
        )

    def detect_cart(
        self,
        image_bgr,
        *,
        captured_at_ns,
        base_motion_epoch,
        head_motion_epoch,
    ):
        return self.detect_scene(
            image_bgr,
            captured_at_ns=captured_at_ns,
            base_motion_epoch=base_motion_epoch,
            head_motion_epoch=head_motion_epoch,
            task=CART_SCENE_TASK,
            profile=CART_SCENE_PROFILE,
            semantic_classes=CART_SCENE_CLASSES,
        )

    def detect_scene(
        self,
        image_bgr,
        *,
        captured_at_ns,
        base_motion_epoch,
        head_motion_epoch,
        task,
        profile,
        semantic_classes,
    ):
        request = self._make_request(
            image_bgr,
            captured_at_ns=captured_at_ns,
            base_motion_epoch=base_motion_epoch,
            head_motion_epoch=head_motion_epoch,
            task=task,
        )
        remaining_s = (request.header.deadline_ns - self._clock_ns()) / 1_000_000_000
        if remaining_s <= 0:
            raise BookVisionError("book_vision_capture_expired")
        try:
            response = self._rpc(request, timeout=min(self.settings.timeout_s, remaining_s))
        except Exception as error:
            raise BookVisionError("book_vision_rpc_failed") from error
        if getattr(response, "task", None) != task:
            raise BookVisionError("book_vision_response_task_mismatch")

        result = []
        for row in getattr(response, "scene_instances", ()):
            if (
                getattr(row, "semantic_class", None) not in semantic_classes
                or getattr(row, "scene_profile_id", None) != profile
            ):
                continue
            bbox = row.bbox_px
            result.append(
                SceneMask(
                    semantic_class=str(row.semantic_class),
                    scene_profile_id=str(row.scene_profile_id),
                    confidence=float(row.confidence),
                    bbox=(
                        int(bbox.x),
                        int(bbox.y),
                        int(bbox.width),
                        int(bbox.height),
                    ),
                    rle_counts=tuple(int(value) for value in row.mask.counts),
                    image_width=int(row.image_width),
                    image_height=int(row.image_height),
                )
            )
        return tuple(sorted(result, key=lambda item: item.confidence, reverse=True))

    def close(self):
        if self._channel is not None:
            self._channel.close()
