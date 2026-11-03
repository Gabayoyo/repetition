# Computer vision file
# Camera init, live frame processing pipeline, conversion to framedata class

import os
import tempfile
from contextlib import contextmanager
from pathlib import Path

import depthai as dai

if not os.environ.get("MPLCONFIGDIR"):
    os.environ["MPLCONFIGDIR"] = str(Path(tempfile.gettempdir()) / "iel-camera-mpl")

import cv2 # or ROS?
import numpy as np

try:
    import mediapipe as mp
except ImportError:
    mp = None

from .dataclass import Frame, Point, StereoFrame
from .models import ensure_model


def sample_depth(depth, x, y, radius=2):
    """Median depth in metres around a normalised (x, y); None where the map is empty."""
    height, width = depth.shape
    px = min(max(int(x * width), 0), width - 1)
    py = min(max(int(y * height), 0), height - 1)
    patch = depth[max(py - radius, 0):py + radius + 1, max(px - radius, 0):px + radius + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) / 1000.0 if valid.size else None


@contextmanager
def capture(fps=30, size=(1280, 720)):
    """Stream StereoFrames: BGR image, aligned depth (mm), confidence, timestamps."""
    with dai.Pipeline() as pipeline:

        # create camera
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        # request the output of camera, gives us queue of frames/input
        queue = cam.requestOutput(
            size, type=dai.ImgFrame.Type.BGR888i, fps=fps
        ).createOutputQueue(maxSize=1, blocking=False)

        # aligns depth onto CAM_A's pixel grid
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.build(True, presetMode=dai.node.StereoDepth.PresetMode.HIGH_DETAIL, size=(640, 480), fps=fps)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(True)

        # separate queues for depth and confidence
        depth_queue = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
        confidence_queue = stereo.confidenceMap.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()

        # core loop
        def frames():
            start = None
            depth = None
            confidence = None
            depth_time = None


            while pipeline.isRunning():

                # get frames
                depth_frame = depth_queue.tryGet()
                confidence_frame = confidence_queue.tryGet()
                frame = queue.tryGet()

                if frame is None:
                    continue

                stamp = frame.getTimestampDevice()
                if start is None:
                    start = stamp

                if depth_frame is not None and confidence_frame is not None:
                    # take snapshot of this specific frame to operate on; otherwise frames move on
                    depth = depth_frame.getFrame().copy()
                    confidence = confidence_frame.getFrame().copy()
                    depth_time = (depth_frame.getTimestampDevice() - start).total_seconds()

                yield StereoFrame(
                    image=frame.getCvFrame(),
                    time_s=(stamp - start).total_seconds(),
                    depth=depth,
                    confidence=confidence,
                    depth_time_s=depth_time,
                )

        stream = frames()

        # using context manager it runs stream
        try:
            yield stream
        finally:
            stream.close() # always called regardless of outcome


@contextmanager
def track():
    """Stream Frames: pose plus both hands, x/y normalised, z metres from stereo depth."""
    if mp is None:
        raise RuntimeError("mediapipe is not installed")

    options = mp.tasks.vision.HolisticLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(ensure_model())),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
    )

    with capture() as capture_frames:
        with mp.tasks.vision.HolisticLandmarker.create_from_options(options) as landmarker:

            def frames():
                last_ms = -1

                # calls capture() infinitely (until exit); yields a StereoFrame
                for stereo in capture_frames:
                    last_ms = max(int(stereo.time_s * 1000), last_ms + 1)
                    rgb = cv2.cvtColor(stereo.image, cv2.COLOR_BGR2RGB)

                    # run model on frame
                    result = landmarker.detect_for_video(
                        mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), last_ms
                    )

                    # one Point per landmark from each of the three landmark sets (hands and body pose)
                    points = [
                        Point(
                            group=group,
                            index=index,
                            x=landmark.x,
                            y=landmark.y,
                            z=sample_depth(stereo.depth, landmark.x, landmark.y) if stereo.depth is not None else None,
                            visibility=landmark.visibility,
                        )
                        for group, landmarks in (
                            ("pose", result.pose_landmarks),
                            ("left_hand", result.left_hand_landmarks),
                            ("right_hand", result.right_hand_landmarks),
                        )
                        for index, landmark in enumerate(landmarks)
                    ] # [ point for i in loop for j in loop2 ] i.e. [ point(j) given j in loop2 ]

                    if not points:
                        continue
                    yield Frame(time_s=stereo.time_s, points=points)

            stream = frames()

            # same format as last time with context manager
            try:
                yield stream
            finally:
                stream.close()
