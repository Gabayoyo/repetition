"""Oak camera handling: pipeline building, frame capture and the frame type."""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time

import depthai as dai
import numpy as np


@dataclass(frozen=True)
class CaptureFrame:
    """One colour frame from the camera."""

    bgr: np.ndarray
    t_s: float
    timestamp_ms: int


# Various camera constants
RGB_SOCKET = dai.CameraBoardSocket.CAM_A
QUEUE_SIZE = 4
READ_TIMEOUT_S = 2.0
USB2_FPS = 15.0  # USB 2 caps the stream at 15 fps
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 360
DEFAULT_FPS = 15.0


def build_pipeline(width: int, height: int, fps: float) -> dai.Pipeline:
    """Builds the depthai pipeline."""
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(RGB_SOCKET)
    camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setInterleaved(False)
    camera.setFps(fps)
    camera.setPreviewSize(width, height)
    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName("rgb")
    camera.preview.link(output.input)
    return pipeline


class OakCapture:
    """USB OAK-D Lite: latest full-field colour preview, with link and rate handling."""

    name = "oak-d-lite"

    def __init__(
        self,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: float = DEFAULT_FPS,
    ) -> None:

        # Check for device
        try:
            self._device = dai.Device()
        except RuntimeError as exc:
            raise RuntimeError(f"no OAK device found over USB ({exc})") from exc

        # Get USB connection metrics, including max fps
        speed = self._device.getUsbSpeed()
        self._report_link(speed)
        capped = self._effective_fps(speed, fps)
        if capped != fps:
            print(
                f"oak: USB 2 caps the stream; using {capped:g} fps",
                file=sys.stderr,
            )

        # Start the built pipeline
        try:
            self._device.startPipeline(build_pipeline(width, height, capped))
        except RuntimeError:
            self._device.close()
            raise

        # Change states now that pipeline has been stopped
        self._queue = self._device.getOutputQueue("rgb", QUEUE_SIZE, blocking=False)
        self._start_ms: int | None = None
        self._last_ms = -1

    @staticmethod
    def _usb3(speed) -> bool:
        return speed in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS)

    @staticmethod
    def _effective_fps(speed, fps: float) -> float:
        return fps if OakCapture._usb3(speed) else min(fps, USB2_FPS)

    def _report_link(self, speed) -> None:
        print(f"oak: {speed.name} link", file=sys.stderr)
        if not self._usb3(speed):
            print(
                "oak: link is not USB 3; bandwidth caps the frame rate",
                file=sys.stderr,
            )

    def read(self) -> CaptureFrame | None:
        """Gets the newest frame"""

        # deadline = current time + the set read timeout duration
        deadline = time.perf_counter() + READ_TIMEOUT_S
        packet = None

        # Get all packets before deadline
        while time.perf_counter() < deadline:
            packets = self._queue.tryGetAll()
            if packets:
                packet = packets[-1] # latest packet out of all of them
                break
            time.sleep(0.001)

        if packet is None:
            print("oak: no colour frames", file=sys.stderr)
            return None
        
        frame = packet.getCvFrame()
        if frame is None:
            return None
        
        timestamp_ms = int(packet.getTimestamp().total_seconds() * 1000)

        if self._start_ms is None:
            self._start_ms = timestamp_ms

        self._last_ms = max(self._last_ms + 1, timestamp_ms)

        # return the CaptureFrame dataclass
        return CaptureFrame(
            bgr=frame,
            t_s=(self._last_ms - self._start_ms) / 1000.0,
            timestamp_ms=self._last_ms,
        )

    def close(self) -> None:
        self._queue.close()
        self._device.close()
