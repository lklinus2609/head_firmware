"""Still pictures of the face from an Intel RealSense color stream, with the
depth stream aligned to it when asked for.

Optional: nothing else in head_ros imports this module, and pyrealsense2,
numpy and Pillow are imported only when a camera is opened. Tested with a
D435i (serial 018322070277, firmware 5.11.1.100, USB 3.2) on 2026-10-18,
pyrealsense2 2.59.1, at 1280 x 720 rgb8 15 fps; depth z16 1280 x 720 15 fps,
1 mm per unit (get_depth_scale() = 0.001 on this camera).
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import threading
import time

# A just-started stream needs some frames before auto exposure settles; 20 at
# 15 fps was enough for a usable picture on 2026-10-18.
WARMUP_FRAMES = 20
FRAME_TIMEOUT_MS = 5000


def list_color_modes(rs=None) -> list:
    """Sorted (width, height, fps, format name) of every color stream mode the
    first RealSense camera offers. RuntimeError when no camera is connected."""
    if rs is None:
        import pyrealsense2 as rs  # optional dependency, see the module docstring
    devices = rs.context().query_devices()
    if len(devices) == 0:
        raise RuntimeError("no RealSense camera found")
    modes = set()
    for sensor in devices[0].query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() != rs.stream.color:
                continue
            video = profile.as_video_stream_profile()
            modes.add((video.width(), video.height(), profile.fps(), str(profile.format())))
    return sorted(modes)


class RealSenseCamera:
    def __init__(self, width: int = 1280, height: int = 720, fps: int = 15,
                 depth: bool = False) -> None:
        import pyrealsense2 as rs  # optional dependency, see the module docstring

        self._rs = rs
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 0, width, height, rs.format.rgb8, fps)
        if depth:
            config.enable_stream(rs.stream.depth, 0, width, height, rs.format.z16, fps)
        profile = self._pipeline.start(config)
        # Depth is mapped onto the color picture, so a pixel means the same
        # place in both.
        self._align = rs.align(rs.stream.color) if depth else None
        self.depth_scale_m = (profile.get_device().first_depth_sensor().get_depth_scale()
                              if depth else None)
        color = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color.get_intrinsics()
        try:
            for _ in range(WARMUP_FRAMES):
                self._pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
        except Exception:
            self._pipeline.stop()
            raise

    def capture(self):
        """(RGB array, depth array in metres or None) of a frame from now."""
        import numpy

        # wait_for_frames returns the latest *unread* set, which can be one the
        # camera made a moment ago; the second call is a frame from now.
        self._pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
        frames = self._pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
        depth_m = None
        if self._align is not None:
            frames = self._align.process(frames)
            raw = numpy.asanyarray(frames.get_depth_frame().get_data())
            depth_m = raw.astype(numpy.float32) * self.depth_scale_m
        image = numpy.asanyarray(frames.get_color_frame().get_data()).copy()
        return image, depth_m

    def deproject(self, x: float, y: float, depth_m: float) -> list:
        """Camera-frame point (metres) of color pixel (x, y) at that depth."""
        return list(self._rs.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth_m))

    def save(self, path: Path, image=None) -> Path:
        """Save a color frame (a new one unless given) as a JPEG; returns the path."""
        from PIL import Image

        if image is None:
            image, _ = self.capture()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image).save(path, quality=90)
        return path

    def poll(self):
        """The newest color frame as an RGB numpy array, or None if no new
        one has arrived (poll_for_frames does not block; an empty frame set
        is false, checked 2026-10-18)."""
        import numpy

        frames = self._pipeline.poll_for_frames()
        if not frames:
            return None
        return numpy.asanyarray(frames.get_color_frame().get_data()).copy()

    def close(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:  # already stopped or unplugged
            pass


class Recorder:
    """Records the camera to an MP4 in a background thread while a run
    goes on, with <video>.frames.csv giving each frame's UTC time (to match
    the telemetry CSV), and saves labelled stills of the newest frame.

    Only the thread touches the camera. OpenCV (opencv-contrib-python, see
    bench/requirements.txt) writes the video; it is imported only here."""

    def __init__(self, camera: RealSenseCamera, video_path: Path, fps: int = 15) -> None:
        import cv2  # optional dependency

        self._cv2 = cv2
        self.camera = camera
        self.video_path = Path(video_path)
        self.video_path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.frames = 0
        self.error: str | None = None
        self._writer = None
        self._index = open(self.video_path.with_suffix(".frames.csv"), "w", encoding="utf-8",
                           buffering=1)
        self._index.write("frame,host_utc\n")
        self._latest = None
        self._latest_s = 0.0          # time.monotonic() when the newest frame came
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="face-recorder", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        cv2 = self._cv2
        try:
            while not self._stop.is_set():
                image = self.camera.poll()
                if image is None:
                    time.sleep(0.005)
                    continue
                stamp = datetime.now(timezone.utc).isoformat()
                with self._lock:
                    self._latest = image
                    self._latest_s = time.monotonic()
                if self._writer is None:
                    height, width = image.shape[:2]
                    self._writer = cv2.VideoWriter(str(self.video_path),
                                                   cv2.VideoWriter_fourcc(*"mp4v"),
                                                   self.fps, (width, height))
                    if not self._writer.isOpened():
                        raise RuntimeError(f"cannot write {self.video_path}")
                self._writer.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                self._index.write(f"{self.frames},{stamp}\n")
                self.frames += 1
        except Exception as error:   # the run goes on without video
            self.error = str(error)

    def latest(self):
        """(time.monotonic() when it came, RGB array) of the newest frame,
        or None before the first."""
        with self._lock:
            return None if self._latest is None else (self._latest_s, self._latest)

    def still(self, path: Path) -> Path | None:
        """Save the newest frame as a JPEG (None if no frame yet)."""
        from PIL import Image

        with self._lock:
            image = self._latest
        if image is None:
            return None
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image).save(path, quality=90)
        return path

    def close(self) -> None:
        """Stop recording; the MP4 becomes a WebM (video_path then names it)."""
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._writer is not None:
            self._writer.release()
        self._index.close()
        self.camera.close()
        if self.frames and self.error is None:
            try:
                self.video_path = to_webm(self.video_path)
            except Exception as error:   # the MP4 stays
                self.error = f"no WebM: {error}"


def to_webm(path: Path) -> Path:
    """Write path (an MP4) again as VP8 WebM next to it and delete the MP4.
    The recorder writes MP4 (mp4v), which this desktop's player cannot open
    (no avdec_mpeg4; GStreamer has vp8dec, checked 2026-10-21)."""
    import cv2

    path = Path(path)
    target = path.with_suffix(".webm")
    source = cv2.VideoCapture(str(path))
    fps = source.get(cv2.CAP_PROP_FPS) or 15
    size = (int(source.get(cv2.CAP_PROP_FRAME_WIDTH)), int(source.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*"VP80"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"cannot write {target}")
    written = 0
    while True:
        ok, frame = source.read()
        if not ok:
            break
        writer.write(frame)
        written += 1
    writer.release()
    source.release()
    if not written or not target.exists() or target.stat().st_size == 0:
        raise RuntimeError(f"{target} is empty")
    path.unlink()
    return target
