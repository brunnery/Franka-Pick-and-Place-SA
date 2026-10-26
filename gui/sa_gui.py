"""SA launcher: stud side switch, one button per command (each opens its own terminal),
and the camera and vision debug images.

  gui/sa_gui            (sources ROS, then runs this file; start from a normal terminal)

Buttons come from gui/gui.yaml. Temporary add-on, not part of the thesis code
(gui/README.md). Without a sourced ROS 2 the buttons work, the image panes stay off.
"""
import argparse
import getpass
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

try:  # ROS ships python_qt_binding (rqt); plain PyQt5 elsewhere
    from python_qt_binding import QtCore, QtGui, QtWidgets
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets

import yaml

import launcher

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
STATUS_POLL_MS = 500
STARTING_S = 5.0          # a click counts as "starting" until the pid file appears
COLOURS = {"idle": "#9e9e9e", "starting": "#f0a020", "running": "#2e9d3a"}
STUD_COLOUR, NUT_COLOUR = QtGui.QColor("#e08a2c"), QtGui.QColor("#5b8bd0")


class RosImages:
    """Latest sensor_msgs/Image per topic, received in a background thread."""

    def __init__(self, topics):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image

        self._rclpy = rclpy
        # No rclpy signal handlers: they would stop only ROS and leave the window hanging;
        # main() closes the window on Ctrl-C / SIGTERM instead
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("sa_gui_viewer")
        self._lock = threading.Lock()
        self._latest = {topic: (None, 0.0) for topic in topics}
        for topic in topics:
            self.node.create_subscription(Image, topic, lambda msg, t=topic: self._store(t, msg),
                                          qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _store(self, topic, msg):
        with self._lock:
            self._latest[topic] = (msg, time.monotonic())

    def latest(self, topic):
        with self._lock:
            return self._latest[topic]

    def shutdown(self):
        self.executor.shutdown()
        self._rclpy.try_shutdown()


def to_qimage(msg):
    """sensor_msgs/Image -> QImage (copy), or None for an unsupported encoding."""
    formats = {"rgb8": QtGui.QImage.Format_RGB888, "bgr8": QtGui.QImage.Format_RGB888,
               "mono8": QtGui.QImage.Format_Grayscale8}
    if msg.encoding not in formats:
        return None
    image = QtGui.QImage(bytes(msg.data), msg.width, msg.height, msg.step,
                         formats[msg.encoding])
    return image.rgbSwapped() if msg.encoding == "bgr8" else image.copy()


class ImagePane(QtWidgets.QGroupBox):
    def __init__(self, title, topic, part=None):
        super().__init__(f"{title}  ({topic})")
        self.topic = topic
        self.part = part      # None = whole image, "left" / "right" = that half only
        self.label = QtWidgets.QLabel("waiting for images")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setMinimumSize(320, 180)
        self.label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.label.setStyleSheet("background: #202020; color: #c0c0c0;")
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.label)
        self._shown_stamp = None

    def show_text(self, text):
        self.label.setPixmap(QtGui.QPixmap())
        self.label.setText(text)
        self._shown_stamp = None

    def update_from(self, ros, stale_s):
        msg, received = ros.latest(self.topic)
        if msg is None:
            return self.show_text(f"no image on {self.topic} yet")
        if time.monotonic() - received > stale_s:
            return self.show_text(f"no image on {self.topic} for {stale_s:.0f} s")
        if received == self._shown_stamp:
            return
        image = to_qimage(msg)
        if image is None:
            return self.show_text(f"unsupported encoding '{msg.encoding}'")
        if self.part is not None:
            half = image.width() // 2
            image = image.copy(half if self.part == "right" else 0, 0, half, image.height())
        pixmap = QtGui.QPixmap.fromImage(image).scaled(
            self.label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.label.setPixmap(pixmap)
        self._shown_stamp = received


class SideSelector(QtWidgets.QWidget):
    """Top view seen from the robot base (+x up, +y left): click an area to put the stud
    there; the nut is in the other area."""

    changed = QtCore.pyqtSignal(str) if hasattr(QtCore, "pyqtSignal") else QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.side = None
        self.coordinates = {}   # side -> (x, y) of its observation pose
        self.setMinimumHeight(180)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setToolTip("Click the area where the stud stands")

    def set_side(self, side, coordinates):
        self.side = side
        self.coordinates = coordinates
        self.update()

    def _areas(self):
        w, h = self.width(), self.height()
        size = QtCore.QSizeF(w * 0.36, h * 0.5)
        left = QtCore.QRectF(QtCore.QPointF(w * 0.06, h * 0.08), size)
        right = QtCore.QRectF(QtCore.QPointF(w * 0.58, h * 0.08), size)
        return {"left": left, "right": right}

    def mousePressEvent(self, event):
        for side, rect in self._areas().items():
            if rect.contains(QtCore.QPointF(event.pos())) and side != self.side:
                self.changed.emit(side)

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        font = p.font()
        font.setBold(True)
        for side, rect in self._areas().items():
            stud = side == self.side
            p.setPen(QtGui.QPen(QtGui.QColor("#404040"), 1.5))
            p.setBrush(STUD_COLOUR if stud else NUT_COLOUR)
            p.drawRoundedRect(rect, 8, 8)
            p.setPen(QtGui.QColor("white"))
            p.setFont(font)
            x, y = self.coordinates.get(side, (float("nan"), float("nan")))
            p.drawText(rect, QtCore.Qt.AlignCenter,
                       f"{'STUD' if stud else 'NUT'}\n{side}\nx {x:.0f}  y {y:+.0f} mm")
        base = QtCore.QPointF(w / 2, h * 0.82)
        p.setPen(QtGui.QPen(QtGui.QColor("#404040"), 1.5))
        p.setBrush(QtGui.QColor("#d0d0d0"))
        p.drawEllipse(base, h * 0.08, h * 0.08)
        p.drawLine(base, QtCore.QPointF(w / 2, h * 0.62))
        p.drawText(QtCore.QRectF(w / 2 + h * 0.12, h * 0.72, w * 0.4, h * 0.2),
                   QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, "robot base, +x up")
        p.end()


class Launcher(QtWidgets.QMainWindow):
    def __init__(self, config_path):
        super().__init__()
        self.cfg = launcher.load_config(config_path)
        self.groups = launcher.groups(self.cfg)
        self.run_dir = Path(tempfile.gettempdir()) / f"sa_gui-{getpass.getuser()}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.poses_file = REPO / self.cfg["poses_file"]
        self.rows = {}          # key -> (entry, dot, stop button)
        self.started = {}       # key -> time of the click
        self.setWindowTitle("SA launcher - FR3 pick, place and screw")
        self.resize(1500, 950)

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(self._controls())
        splitter.addWidget(self._viewers())
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([560, 940])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage(f"Commands run from {REPO}, buttons from {config_path}")

        self._refresh_side()
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._poll)
        timer.start(STATUS_POLL_MS)

    # --- left side: stud side and buttons -------------------------------------------
    def _controls(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        side_box = QtWidgets.QGroupBox("Stud side (seen from the robot base)")
        side_layout = QtWidgets.QVBoxLayout(side_box)
        self.selector = SideSelector()
        self.selector.changed.connect(self._set_side)
        self.side_label = QtWidgets.QLabel()
        self.side_label.setWordWrap(True)
        side_layout.addWidget(self.selector)
        side_layout.addWidget(self.side_label)
        layout.addWidget(side_box)

        for name, entries in self.groups:
            box = QtWidgets.QGroupBox(name)
            grid = QtWidgets.QGridLayout(box)
            grid.setHorizontalSpacing(4)
            for i, entry in enumerate(entries):
                row, column = divmod(i, 2)
                grid.addWidget(self._button_row(entry), row, column)
            layout.addWidget(box)

        stop_all = QtWidgets.QPushButton("Stop all running (Ctrl-C to each)")
        stop_all.clicked.connect(self._stop_all)
        layout.addWidget(stop_all)
        layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(520)
        return scroll

    def _button_row(self, entry):
        row = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        dot = QtWidgets.QLabel("●")
        dot.setFixedWidth(14)
        button = QtWidgets.QPushButton(entry.label.replace("&", "&&"))  # no mnemonic
        tip = f"{entry.tip}\n\n$ {entry.command}" if entry.tip else f"$ {entry.command}"
        button.setToolTip(tip)
        button.clicked.connect(lambda _=False, e=entry: self._start(e))
        stop = QtWidgets.QToolButton()
        stop.setText("■")
        stop.setToolTip("Stop (Ctrl-C)")
        stop.clicked.connect(lambda _=False, e=entry: self._stop(e))
        layout.addWidget(dot)
        layout.addWidget(button, 1)
        layout.addWidget(stop)
        self.rows[entry.key] = (entry, dot, stop)
        self._show_state(entry.key, "idle")
        return row

    # --- right side: images ----------------------------------------------------------
    def _viewers(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        self.panes = [ImagePane(v["title"], v["topic"], v.get("part"))
                      for v in self.cfg["viewers"]]
        for pane in self.panes:
            layout.addWidget(pane, 1)
        try:
            self.ros = RosImages(sorted({pane.topic for pane in self.panes}))
        except Exception as error:  # noqa: BLE001 - no ROS sourced, buttons still useful
            self.ros = None
            for pane in self.panes:
                pane.show_text(f"image view off: ROS 2 not available ({error})")
        else:
            timer = QtCore.QTimer(self)
            timer.timeout.connect(self._draw_images)
            timer.start(int(1000 / self.cfg["viewer_rate_hz"]))
        return panel

    def _draw_images(self):
        for pane in self.panes:
            pane.update_from(self.ros, self.cfg["viewer_stale_s"])

    # --- stud side -------------------------------------------------------------------
    def _refresh_side(self):
        try:
            poses = yaml.safe_load(self.poses_file.read_text(encoding="utf-8"))
            side = launcher.stud_side(poses)
            xy = launcher.area_xy(poses)
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.side_label.setText(f"cannot read the observation poses: {error}")
            return
        other = "right" if side == "left" else "left"
        coordinates = {side: xy[launcher.STUD_POSE], other: xy[launcher.NUT_POSE]}
        if (side, coordinates) != (self.selector.side, self.selector.coordinates):
            self.selector.set_side(side, coordinates)
            self.side_label.setText(
                f"Stud <b>{side}</b>, nut <b>{other}</b>. Click the other area to swap: the x/y "
                f"of observe_nut and observe_stud are exchanged in {self.cfg['poses_file']} "
                "(z stays). Used by every later run, also from the terminal.")

    def _set_side(self, side):
        try:
            self.poses_file.write_text(
                launcher.swap_areas(self.poses_file.read_text(encoding="utf-8")),
                encoding="utf-8")
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            QtWidgets.QMessageBox.critical(self, "Stud side", f"not changed: {error}")
            return
        self._refresh_side()
        busy = [e.label for e in self._running() if e.confirm]
        note = f" ({', '.join(busy)} still uses the old poses)" if busy else ""
        self.statusBar().showMessage(f"Stud now {side}{note}")

    # --- running commands ------------------------------------------------------------
    def _pid_file(self, entry):
        return self.run_dir / f"{entry.key}.pid"

    def _running(self):
        return [e for e, *_ in self.rows.values()
                if launcher.running_pid(self._pid_file(e)) or self._starting(e.key)]

    def _start(self, entry):
        if launcher.running_pid(self._pid_file(entry)) or self._starting(entry.key):
            self.statusBar().showMessage(f"'{entry.label}' is already running")
            return
        clashes = launcher.conflicts(entry, self._running())
        if clashes:
            text = "\n".join(f"- {label} (uses the {resource})" for label, resource in clashes)
            answer = QtWidgets.QMessageBox.warning(
                self, entry.label, f"Already running:\n{text}\n\nTwo drivers on the same "
                "hardware fight each other. Stop those first, or start anyway?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel)
            if answer != QtWidgets.QMessageBox.Yes:
                return
        if entry.requires and not shutil.which(entry.requires):
            QtWidgets.QMessageBox.warning(self, entry.label,
                                          f"'{entry.requires}' is not installed.")
            return
        if entry.confirm:
            answer = QtWidgets.QMessageBox.question(
                self, entry.label, f"{self.cfg['robot_confirm_text']}\n\nStart '{entry.label}'?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Cancel)
            if answer != QtWidgets.QMessageBox.Yes:
                return
        script = self.run_dir / f"{entry.key}.sh"
        script.write_text(launcher.shell_script(entry, self.cfg, REPO, self._pid_file(entry)),
                          encoding="utf-8")
        try:
            command = launcher.terminal_command(self.cfg, entry.label, script)
            subprocess.Popen(command, cwd=REPO, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, RuntimeError) as error:
            QtWidgets.QMessageBox.critical(self, entry.label, f"could not start: {error}")
            return
        self.started[entry.key] = time.monotonic()
        self._show_state(entry.key, "starting")
        self.statusBar().showMessage(f"started '{entry.label}' in a new terminal")

    def _starting(self, key):
        return time.monotonic() - self.started.get(key, -STARTING_S) < STARTING_S

    def _stop(self, entry):
        pid = launcher.running_pid(self._pid_file(entry))
        if pid is None:
            return
        try:
            launcher.interrupt(pid)
            self.statusBar().showMessage(f"Ctrl-C sent to '{entry.label}'")
        except OSError as error:
            self.statusBar().showMessage(f"could not stop '{entry.label}': {error}")

    def _stop_all(self):
        for entry, *_ in self.rows.values():
            self._stop(entry)

    def _show_state(self, key, state):
        _, dot, stop = self.rows[key]
        dot.setStyleSheet(f"color: {COLOURS[state]}; font-size: 14px;")
        dot.setToolTip(state)
        stop.setEnabled(state == "running")

    def _poll(self):
        for key, (entry, *_) in self.rows.items():
            if launcher.running_pid(self._pid_file(entry)):
                self.started.pop(key, None)
                self._show_state(key, "running")
            else:
                self._show_state(key, "starting" if self._starting(key) else "idle")
        self._refresh_side()

    def closeEvent(self, event):
        if self.ros is not None:
            self.ros.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=HERE / "gui.yaml")
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = Launcher(args.config)
    window.show()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: window.close())
    # Qt's event loop does not run Python code on its own; the timer lets the handler run
    wake = QtCore.QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(STATUS_POLL_MS)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
