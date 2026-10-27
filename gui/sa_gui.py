"""SA launcher: stud side switch, one button per command (each opens its own terminal),
the run timer, the rating of the last run, and the camera and vision debug images.

  gui/sa_gui            (sources ROS, then runs this file; start from a normal terminal)

Buttons come from gui/gui.yaml. Temporary add-on, not part of the thesis code
(gui/README.md). Without a sourced ROS 2 the buttons work, image panes and timer stay off.
"""
import argparse
import getpass
import json
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
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
    """Latest sensor_msgs/Image per topic and the run events of sa_tasks (JSON in
    std_msgs/String), received in a background thread."""

    def __init__(self, topics, event_topic, event_history):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                               qos_profile_sensor_data)
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

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
        self._events = []  # (event dict, local time received)
        # as the publisher (tasks.yaml run_record): marks sent before the GUI connected arrive too
        self.node.create_subscription(String, event_topic, self._store_event, QoSProfile(
            depth=event_history, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _store(self, topic, msg):
        with self._lock:
            self._latest[topic] = (msg, time.monotonic())

    def latest(self, topic):
        with self._lock:
            return self._latest[topic]

    def _store_event(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._events.append((event, time.monotonic()))

    def pop_events(self):
        with self._lock:
            events, self._events = self._events, []
        return events

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

        self.run_timer = launcher.RunTimer(self.cfg["timer"]["end_event"],
                                           self.cfg["timer"]["stop_event"])
        self.rated = None       # (record path, mtime) shown in the rating box

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(self._controls())
        splitter.addWidget(self._viewers())
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([560, 940])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage(f"Commands run from {REPO}, buttons from {config_path}")

        self._refresh_side()
        self._refresh_rating()
        self._refresh_counter()
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._poll)
        timer.start(STATUS_POLL_MS)
        if self.ros is not None:
            clock = QtCore.QTimer(self)
            clock.timeout.connect(self._update_timer)
            clock.start(self.cfg["timer"]["refresh_ms"])
        else:
            self.timer_total.setText("off")
            self.timer_info.setText("Timer off: ROS 2 not available")

    # --- left side: stud side and buttons -------------------------------------------
    def _controls(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.addWidget(self._timer_box())
        layout.addWidget(self._rating_box())

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

    # --- run timer ---------------------------------------------------------------------
    def _timer_box(self):
        t = self.cfg["timer"]
        box = QtWidgets.QGroupBox("Run timer (from robot, gripper, vision up to the iris opening)")
        grid = QtWidgets.QGridLayout(box)
        self.timer_total = QtWidgets.QLabel("--")
        font = self.timer_total.font()
        font.setPointSize(t["font_pt"])
        font.setBold(True)
        self.timer_total.setFont(font)
        grid.addWidget(self.timer_total, 0, 0, 1, 2)
        font.setPointSize(t["split_font_pt"])
        self.timer_splits = {}
        for row, split in enumerate(t["splits"], 1):
            name = QtWidgets.QLabel(split["label"])
            value = QtWidgets.QLabel("--")
            name.setFont(font)
            value.setFont(font)
            grid.addWidget(name, row, 0)
            grid.addWidget(value, row, 1)
            self.timer_splits[split["event"]] = value
        self.timer_info = QtWidgets.QLabel("no run yet")
        self.timer_info.setWordWrap(True)
        self.timer_info.setStyleSheet("color: #606060;")
        grid.addWidget(self.timer_info, len(t["splits"]) + 1, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        return box

    def _update_timer(self):
        for event, received in self.ros.pop_events():
            try:
                self.run_timer.feed(event, received)
            except (KeyError, TypeError, ValueError):
                continue
        timer = self.run_timer
        elapsed = timer.elapsed(time.monotonic())
        if elapsed is None:
            return
        self.timer_total.setText(f"{elapsed:6.1f} s")
        previous = None
        for split in self.cfg["timer"]["splits"]:
            t = timer.marks.get(split["event"])
            text = "--" if t is None else f"{t:.1f} s"
            if t is not None and previous is not None:
                text += f"   (+{t - previous:.1f} s)"
            self.timer_splits[split["event"]].setText(text)
            previous = t if t is not None else previous
        steps, previous = [], 0.0  # time per step (since the mark before)
        for name, t in timer.marks.items():
            steps.append(f"{name} {t - previous:.1f}")
            previous = t
        marks = "  ·  ".join(steps)
        if timer.finished:
            state = (f"finished: {timer.finished.get('result')}, "
                     f"record {timer.finished.get('record')}")
        else:
            state = "running"
        self.timer_info.setText(f"{timer.step or ''} {timer.run or ''}: {state}\n{marks}")

    # --- rating of the last run --------------------------------------------------------
    def _rating_box(self):
        o = self.cfg["outcome"]
        box = QtWidgets.QGroupBox("Rate the last run (written into its record)")
        grid = QtWidgets.QGridLayout(box)
        self.round_counter = launcher.RoundCounter()
        self.round_label = QtWidgets.QLabel("")
        font = self.round_label.font()
        font.setPointSize(self.cfg["counter"]["font_pt"])
        font.setBold(True)
        self.round_label.setFont(font)
        grid.addWidget(self.round_label, 0, 0, 1, 4)
        self.round_details = QtWidgets.QLabel("")
        grid.addWidget(self.round_details, 1, 0, 1, 4)
        self.rating_record = QtWidgets.QLabel("no record yet")
        self.rating_record.setWordWrap(True)
        grid.addWidget(self.rating_record, 2, 0, 1, 4)
        top = 3  # rows above the form
        self.rating_buttons = QtWidgets.QButtonGroup(box)
        for column, (value, label) in enumerate(o["screwed"].items()):
            button = QtWidgets.QRadioButton(label)
            button.setProperty("value", value)
            self.rating_buttons.addButton(button)
            grid.addWidget(button, top, column)
        grid.addWidget(QtWidgets.QLabel("Turns"), top + 1, 0)
        self.rating_turns = QtWidgets.QDoubleSpinBox()
        self.rating_turns.setRange(-o["turns_step"], o["turns_max"])
        self.rating_turns.setSingleStep(o["turns_step"])
        self.rating_turns.setDecimals(1)
        self.rating_turns.setSpecialValueText("not counted")  # shown at the minimum
        grid.addWidget(self.rating_turns, top + 1, 1)
        grid.addWidget(QtWidgets.QLabel("Failed at"), top + 1, 2)
        self.rating_phase = QtWidgets.QComboBox()
        self.rating_phase.addItems(o["phases"])
        grid.addWidget(self.rating_phase, top + 1, 3)
        self.rating_note = QtWidgets.QLineEdit()
        self.rating_note.setPlaceholderText("note (optional)")
        grid.addWidget(self.rating_note, top + 2, 0, 1, 3)
        save = QtWidgets.QPushButton("Save rating")
        save.clicked.connect(self._save_rating)
        grid.addWidget(save, top + 2, 3)
        return box

    def _refresh_counter(self):
        """Run counter of the current test round (run_record.round in tasks.yaml)."""
        c = self.cfg["counter"]
        try:
            tasks = yaml.safe_load((REPO / c["tasks_file"]).read_text(encoding="utf-8"))
            round_ = tasks["run_record"]["round"]
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.round_label.setText(f"run counter off: {error}")
            return
        counts = self.round_counter.count(sorted(REPO.glob(self.cfg["outcome"]["records"])),
                                          round_)
        total = sum(counts.values())
        self.round_label.setText(f"{round_}: run {total} / {c['target_runs']}")
        rated = counts["yes"] + counts["partly"] + counts["no"]
        rate = f", {100.0 * counts['yes'] / rated:.0f} % of the rated" if rated else ""
        self.round_details.setText(
            f"screwed on {counts['yes']} · partly {counts['partly']} · failed {counts['no']} · "
            f"aborted {counts['aborted']} · not rated {counts['unrated']}{rate}")

    def _reset_rating(self):
        self.rating_buttons.setExclusive(False)
        for button in self.rating_buttons.buttons():
            button.setChecked(False)
        self.rating_buttons.setExclusive(True)
        self.rating_turns.setValue(self.rating_turns.minimum())
        self.rating_phase.setCurrentIndex(0)
        self.rating_note.clear()

    def _refresh_rating(self):
        path = launcher.latest_record(REPO, self.cfg["outcome"]["records"])
        if path is None:
            return
        try:
            stamp = (path, path.stat().st_mtime)
            if stamp == self.rated:
                return
            record = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            self.rating_record.setText(f"cannot read {path.name}: {error}")
            return
        if self.rated is None or self.rated[0] != path:
            self._reset_rating()  # a new run: empty form
        self.rated = stamp
        outcome = record.get(launcher.OUTCOME_KEY)
        rating = "not rated yet" if not outcome else (
            f"rated: {outcome.get('screwed')}, turns {outcome.get('turns')}, "
            f"failed at {outcome.get('failed_phase')}")
        self.rating_record.setText(f"<b>{path.name}</b>: {record.get('result')}, "
                                   f"{record.get('duration_s')} s. {rating}")

    def _save_rating(self):
        button = self.rating_buttons.checkedButton()
        path = self.rated[0] if self.rated else None
        if button is None or path is None:
            self.statusBar().showMessage("choose screwed on / partly / failed first")
            return
        turns = self.rating_turns.value()
        outcome = {"screwed": button.property("value"),
                   "turns": None if turns == self.rating_turns.minimum() else turns,
                   "failed_phase": self.rating_phase.currentText(),
                   "note": self.rating_note.text().strip(),
                   "rated": datetime.now().isoformat(timespec="seconds")}
        try:
            path.write_text(launcher.set_outcome(path.read_text(encoding="utf-8"), outcome),
                            encoding="utf-8")
        except OSError as error:
            QtWidgets.QMessageBox.critical(self, "Rating", f"not saved: {error}")
            return
        self.statusBar().showMessage(f"rating saved in {path.name}")
        self._refresh_rating()

    # --- right side: images ----------------------------------------------------------
    def _viewers(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        self.panes = [ImagePane(v["title"], v["topic"], v.get("part"))
                      for v in self.cfg["viewers"]]
        for pane in self.panes:
            layout.addWidget(pane, 1)
        try:
            self.ros = RosImages(sorted({pane.topic for pane in self.panes}),
                                 self.cfg["timer"]["topic"], self.cfg["timer"]["history"])
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
        self._refresh_rating()
        self._refresh_counter()

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
