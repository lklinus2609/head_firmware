"""Live picture from the RealSense, to aim it at the face.

    scripts/face-preview
    scripts/face-preview --list-modes
    scripts/face-preview --mode WIDTHxHEIGHT@FPS     (a mode from --list-modes)

Close it before a head-sweep run with --camera realsense: only one program
can open the camera. S saves the current picture next to the tuning records.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import signal
import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QShortcut

try:
    from .face_camera import RealSenseCamera, list_color_modes
except ImportError:  # run as a plain script from this directory
    from face_camera import RealSenseCamera, list_color_modes  # type: ignore[no-redef]

RECORDS = Path(__file__).resolve().parents[4] / "tuning_runs"


class PreviewWindow(QMainWindow):
    def __init__(self, camera: RealSenseCamera) -> None:
        super().__init__()
        self.camera = camera
        self.frame = None
        self.setWindowTitle("Face camera - S saves a picture")
        self.label = QLabel("Waiting for the camera...")
        self.label.setAlignment(Qt.AlignCenter)
        self.setCentralWidget(self.label)
        self.resize(1280, 760)
        QShortcut(QKeySequence("S"), self, activated=self.save)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(20)

    def refresh(self) -> None:
        frame = self.camera.poll()
        if frame is None:
            return
        self.frame = frame
        height, width, _ = frame.shape
        image = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888)
        self.label.setPixmap(QPixmap.fromImage(image).scaled(
            self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def save(self) -> None:
        if self.frame is None:
            return
        from PIL import Image

        now = datetime.now()               # local time, like the run folders
        path = RECORDS / "runs" / f"{now:%Y-%m-%d}" / f"{now:%H%M%S}_camera.jpg"
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(self.frame).save(path, quality=90)
        self.setWindowTitle(f"Face camera - saved {path.name}")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        self.timer.stop()
        self.camera.close()
        super().closeEvent(event)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list-modes", action="store_true",
                        help="print the camera's color modes (width, height, fps, format) and exit")
    parser.add_argument("--mode", default="1280x720@15", metavar="WxH@FPS",
                        help="color mode to show (default 1280x720@15)")
    a = parser.parse_args()
    if a.list_modes:
        try:
            modes = list_color_modes()
        except RuntimeError as error:
            print(f"face-preview: {error}")
            sys.exit(1)
        for width, height, fps, name in modes:
            print(f"{width}x{height}@{fps} {name}")
        return
    size, _, fps = a.mode.partition("@")
    width, _, height = size.partition("x")
    application = QApplication(sys.argv[:1])
    window = PreviewWindow(RealSenseCamera(int(width), int(height), int(fps)))
    signal.signal(signal.SIGINT, lambda _signal, _frame: window.close())
    idle = QTimer()
    idle.timeout.connect(lambda: None)
    idle.start(100)
    window.show()
    sys.exit(application.exec_())


if __name__ == "__main__":
    main()
