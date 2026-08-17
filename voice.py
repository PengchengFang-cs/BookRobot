"""语音积木：把一句话变成 banana / apple / orange ..."""

import json
import struct
import subprocess
import time
import wave
from collections import deque
from pathlib import Path

import rclpy
from std_msgs.msg import String

from config import (
    TEXT_COMMAND_TOPIC,
    VOICE_TOPIC,
    WHISPER_CHANNELS,
    WHISPER_MIN_PEAK,
    WHISPER_MODEL,
    WHISPER_RECORD_SECONDS,
    WHISPER_WAV,
)
from geometry import fruit_from_text


def _all_text(value):
    """把任意 JSON 里的文字都拿出来。"""
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        result = []
        for item in value.values():
            result.extend(_all_text(item))
        return result
    if isinstance(value, list):
        result = []
        for item in value:
            result.extend(_all_text(item))
        return result
    return []


class Voice:
    def __init__(self, node):
        self.node = node
        self.commands = deque()
        self.whisper_model = None
        self.whisper_disabled = False
        self.audio_status = None
        self.audio_event = 0
        self.node.create_subscription(String, TEXT_COMMAND_TOPIC, self._text_callback, 10)

        self.tts_client = None
        self.mode_client = None
        try:
            from ros2_comm.msg import AudioTopic
            from ros2_comm.srv import RobotSrv, TxtSrv

            self.AudioTopic = AudioTopic
            self.RobotSrv = RobotSrv
            self.TxtSrv = TxtSrv
            self.node.create_subscription(AudioTopic, VOICE_TOPIC, self._voice_callback, 10)
            self.tts_client = self.node.create_client(TxtSrv, "/aiui_pkg/txt_service")
            self.mode_client = self.node.create_client(
                RobotSrv, "/aiui_pkg/robot_service"
            )
        except ImportError:
            self.AudioTopic = None
            self.RobotSrv = None
            self.TxtSrv = None
            self.node.get_logger().warning("没有找到 AIUI 消息，仍可使用文字 topic")

    def add_text(self, text):
        fruit = fruit_from_text(text)
        if fruit and fruit not in self.commands:
            print(f"[语音] 听到: {text} -> {fruit}")
            self.commands.append(fruit)

    def _text_callback(self, message):
        self.add_text(message.data)

    def _voice_callback(self, message):
        # 记录每次“播放/停止”变化，让 say() 能等自己的提示音播完。
        self.audio_status = message.status
        self.audio_event += 1

        # status=1 表示 AIUI 完成了一次识别。
        if message.status != 1:
            return
        description = message.description.strip()
        if not description:
            return

        # 这台机器人有时把字段说明放在 description 里，不是识别文字。
        if description == "0-唤醒 1-识别 2-播放 3-停止":
            return

        # 即使 JSON 不完整，直接在原文里找“香蕉”等词也能工作。
        self.add_text(description)
        try:
            texts = _all_text(json.loads(description))
        except (ValueError, TypeError):
            return
        for text in texts:
            self.add_text(text)

    def _interaction_mode(self, enter):
        """让 AIUI 开始或停止听人说话。"""
        if self.mode_client is None or not rclpy.ok():
            return
        try:
            if not self.mode_client.wait_for_service(timeout_sec=1.0):
                return

            request = self.RobotSrv.Request()
            request.cmd = 3
            request.param = 1 if enter else 2
            future = self.mode_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            result = future.result() if future.done() else None
            if result is not None and result.robot_reply == 0:
                print("[语音] 开始听指令" if enter else "[语音] 停止听指令")
        except Exception as error:
            # Ctrl+C 可能先关闭 ROS context，finally 仍会走到这里。
            if rclpy.ok():
                self.node.get_logger().warning(f"切换 AIUI 模式失败: {error}")

    def _load_whisper(self):
        """第一次听指令时才加载模型，之后一直复用。"""
        if self.whisper_disabled:
            return None
        if self.whisper_model is not None:
            return self.whisper_model

        model_file = Path.home() / ".cache" / "whisper" / f"{WHISPER_MODEL}.pt"
        if not model_file.exists():
            self.node.get_logger().warning(f"没有本地 Whisper 模型: {model_file}")
            self.whisper_disabled = True
            return None
        try:
            import whisper

            print(f"[语音] 加载本地 Whisper {WHISPER_MODEL} 模型……")
            self.whisper_model = whisper.load_model(WHISPER_MODEL)
            return self.whisper_model
        except Exception as error:
            self.node.get_logger().warning(f"Whisper 不能启动: {error}")
            self.whisper_disabled = True
            return None

    def prepare(self):
        """先加载模型；听到“准备好”以后再说，就不会漏掉第一句话。"""
        self._load_whisper()

    def _listen_with_whisper(self):
        """从默认麦克风录几秒，只在本机转成文字。"""
        model = self._load_whisper()
        if model is None:
            return

        wav_path = Path(WHISPER_WAV)
        wav_path.unlink(missing_ok=True)
        print(f"[语音] 请在 {WHISPER_RECORD_SECONDS:.0f} 秒内说水果名字……")
        command = (
            "timeout",
            "--signal=INT",
            f"{WHISPER_RECORD_SECONDS}s",
            "parec",
            "--record",
            "--device=@DEFAULT_SOURCE@",
            "--rate=16000",
            "--format=s16le",
            f"--channels={WHISPER_CHANNELS}",
            "--file-format=wav",
            str(wav_path),
        )
        subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if not wav_path.exists() or wav_path.stat().st_size <= 44:
            self.node.get_logger().warning("麦克风没有录到声音")
            return

        # 太安静时 Whisper 容易凭空猜一句话；冒烟测试直接重录最简单。
        try:
            with wave.open(str(wav_path), "rb") as wav:
                sound = wav.readframes(wav.getnframes())
            peak = max(
                (abs(value[0]) for value in struct.iter_unpack("<h", sound)),
                default=0,
            )
        except (OSError, wave.Error, struct.error) as error:
            self.node.get_logger().warning(f"录音不能读取: {error}")
            return
        if peak < WHISPER_MIN_PEAK:
            print("[语音] 没听清，请再说一次")
            return

        try:
            result = model.transcribe(
                str(wav_path),
                language="zh",
                fp16=False,
                temperature=0,
                # 这里只听一个水果名字，不需要生成时间轴或很长的句子。
                without_timestamps=True,
                sample_len=16,
                initial_prompt="机器人指令：香蕉、苹果、橙子、梨。",
                condition_on_previous_text=False,
            )
        except Exception as error:
            self.node.get_logger().warning(f"Whisper 识别失败: {error}")
            return
        text = result.get("text", "").strip()
        if text:
            print(f"[语音] Whisper 文字: {text}")
            self.add_text(text)

    def wait_for_fruit(self):
        print("\n请说：香蕉、苹果、橙子或梨。")
        print("也可以发布文字：ros2 topic pub --once /fruit_test/command "
              "std_msgs/msg/String \"{data: 香蕉}\"")
        self._interaction_mode(True)
        try:
            while rclpy.ok():
                if self.commands:
                    return self.commands.popleft()
                # 先给 AIUI 和文字 topic 一点时间；没有文字时再用本地 Whisper。
                for _ in range(5):
                    rclpy.spin_once(self.node, timeout_sec=0.10)
                    if self.commands:
                        return self.commands.popleft()
                self._listen_with_whisper()
            raise KeyboardInterrupt
        finally:
            self._interaction_mode(False)

    def say(self, text):
        print(f"[机器人] {text}")
        if self.tts_client is None or not rclpy.ok():
            return
        if not self.tts_client.wait_for_service(timeout_sec=1.0):
            return

        request = self.TxtSrv.Request()
        request.cmd = 1
        request.txt_content = text
        first_event = self.audio_event
        try:
            self.tts_client.call_async(request)
        except Exception as error:
            if rclpy.ok():
                self.node.get_logger().warning(f"语音播报失败: {error}")
            return

        # TxtSrv 会在扬声器播完之前返回，所以必须等 AudioTopic 的 2 -> 3。
        heard_playing = False
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.10)
            if self.audio_event <= first_event:
                continue
            if self.audio_status == 2:
                heard_playing = True
            elif heard_playing and self.audio_status == 3:
                return
        if rclpy.ok():
            self.node.get_logger().warning("没有等到 AIUI 播报结束，继续运行")
