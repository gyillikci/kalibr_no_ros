"""Run, pause, resume and cancel a Kalibr command from outside its process.

Kalibr's work moves between extraction processes, single-threaded Python and
a blocking C++ optimizer that holds the GIL, so no thread inside Kalibr can be
relied on to receive a request. The supervisor instead starts the command in
its own session and signals the whole process group, which reaches every
process and thread Kalibr creates without knowing how many there are:

- pause:  SIGSTOP to the group
- resume: SIGCONT to the group
- cancel: SIGTERM to the group, SIGKILL after a timeout

Progress is parsed from the command's output. POSIX only.
"""

import enum
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence


class RunState(enum.Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    CANCELLING = "cancelling"
    CANCELLED = "cancelled"
    SUCCEEDED = "succeeded"
    FAILED = "failed"

    @property
    def finished(self):
        return self in (RunState.CANCELLED, RunState.SUCCEEDED, RunState.FAILED)


class InvalidTransition(RuntimeError):
    pass


@dataclass(frozen=True)
class Progress:
    phase: str = "starting"
    done: Optional[int] = None
    total: Optional[int] = None
    iteration: Optional[int] = None

    def describe(self):
        text = self.phase
        if self.done is not None and self.total:
            text += " {}/{}".format(self.done, self.total)
        if self.iteration is not None:
            text += " (iteration {})".format(self.iteration)
        return text


# First match wins; phases follow the order Kalibr prints them in.
_PHASES = [
    (re.compile(r"Extracting calibration target corners"), "extracting corners"),
    (re.compile(r"Building the problem"), "building problem"),
    (re.compile(r"Estimating time shift camera to imu"), "initializing"),
    (re.compile(r"Optimization problem initialized"), "optimizing"),
    (re.compile(r"Starting final outlier filtering"), "filtering outliers"),
    (re.compile(r"Recovering covariance"), "recovering covariance"),
    (re.compile(r"After Optimization|Calibration complete"), "writing results"),
    (re.compile(r"Generating result report"), "writing report"),
]
_PROGRESS = re.compile(r"Progress (\d+) / (\d+)")
_ITERATION = re.compile(r"^\[(\d+)\]: J:")


def parse_progress(line, current):
    """Return the progress after one output line."""
    for pattern, phase in _PHASES:
        if pattern.search(line):
            return Progress(phase=phase)
    match = _PROGRESS.search(line)
    if match:
        return Progress(current.phase, int(match.group(1)), int(match.group(2)), current.iteration)
    match = _ITERATION.match(line)
    if match:
        return Progress(current.phase, current.done, current.total, int(match.group(1)))
    return current


class CalibrationSupervisor:
    """Owns the state of one Kalibr run; all methods are thread-safe."""

    def __init__(
        self,
        argv: Sequence[str],
        *,
        cwd=None,
        env=None,
        log_path=None,
        on_state: Optional[Callable[[RunState], None]] = None,
        on_output: Optional[Callable[[str], None]] = None,
        on_progress: Optional[Callable[[Progress], None]] = None,
    ):
        self.argv: List[str] = list(argv)
        self.cwd = cwd
        self.env = env
        self.log_path = log_path
        self._on_state = on_state
        self._on_output = on_output
        self._on_progress = on_progress
        self._lock = threading.RLock()
        self._finished = threading.Event()
        self._state = RunState.IDLE
        self._progress = Progress()
        self._process: Optional[subprocess.Popen] = None
        self._cancel_requested = False
        self._started_at = None
        self._ended_at = None
        self._paused_since = None
        self._paused_total = 0.0

    @property
    def state(self) -> RunState:
        return self._state

    @property
    def progress(self) -> Progress:
        return self._progress

    @property
    def returncode(self) -> Optional[int]:
        return self._process.returncode if self._process else None

    @property
    def pid(self) -> Optional[int]:
        return self._process.pid if self._process else None

    def active_seconds(self) -> float:
        """Wall time spent running, not counting pauses."""
        with self._lock:
            if self._started_at is None:
                return 0.0
            if self._ended_at is not None:
                end = self._ended_at
            elif self._paused_since is not None:
                end = self._paused_since
            else:
                end = time.monotonic()
            return end - self._started_at - self._paused_total

    def start(self):
        with self._lock:
            self._require(RunState.IDLE)
            self._process = subprocess.Popen(
                self.argv,
                cwd=self.cwd,
                env=self.env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,  # own process group, so signals reach all children
            )
            self._started_at = time.monotonic()
            self._set_state(RunState.RUNNING)
        threading.Thread(target=self._pump, name="kalibr-output", daemon=True).start()
        return self

    def pause(self):
        with self._lock:
            self._require(RunState.RUNNING)
            self._signal_group(signal.SIGSTOP)
            self._paused_since = time.monotonic()
            self._set_state(RunState.PAUSED)

    def resume(self):
        with self._lock:
            self._require(RunState.PAUSED)
            self._signal_group(signal.SIGCONT)
            self._paused_total += time.monotonic() - self._paused_since
            self._paused_since = None
            self._set_state(RunState.RUNNING)

    def cancel(self, timeout: float = 10.0):
        """Terminate the run; escalate to SIGKILL if it outlives ``timeout``."""
        with self._lock:
            if self._state in (RunState.CANCELLING,) or self._state.finished:
                return
            self._require(RunState.RUNNING, RunState.PAUSED)
            self._cancel_requested = True
            self._signal_group(signal.SIGTERM)
            if self._paused_since is not None:
                # A stopped process only acts on SIGTERM once it is continued.
                self._signal_group(signal.SIGCONT)
                self._paused_total += time.monotonic() - self._paused_since
                self._paused_since = None
            self._set_state(RunState.CANCELLING)
        if not self._finished.wait(timeout):
            self._signal_group(signal.SIGKILL)

    def wait(self, timeout: Optional[float] = None) -> RunState:
        self._finished.wait(timeout)
        return self._state

    def _require(self, *allowed):
        if self._state not in allowed:
            raise InvalidTransition(
                "cannot do that while {}; allowed only when {}".format(
                    self._state.value, " or ".join(s.value for s in allowed)
                )
            )

    def _signal_group(self, signum):
        try:
            os.killpg(self._process.pid, signum)
        except ProcessLookupError:
            pass  # the group has already exited

    def _set_state(self, state):
        self._state = state
        if self._on_state:
            self._on_state(state)

    def _pump(self):
        log = open(self.log_path, "a", encoding="utf-8") if self.log_path else None
        try:
            buffer = b""
            while True:
                chunk = self._process.stdout.read1(65536)
                if not chunk:
                    break
                if log:
                    log.write(chunk.decode("utf-8", "replace"))
                    log.flush()
                buffer += chunk
                # Kalibr redraws progress with '\r'; treat it as a line end.
                *lines, buffer = re.split(rb"\r\n|\r|\n", buffer)
                for raw in lines:
                    self._handle_line(raw.decode("utf-8", "replace"))
            if buffer:
                self._handle_line(buffer.decode("utf-8", "replace"))
        finally:
            if log:
                log.close()
            self._process.stdout.close()
            self._process.wait()
            # Sweep any child left in the group (e.g. orphaned workers).
            self._signal_group(signal.SIGKILL)
            with self._lock:
                self._ended_at = time.monotonic()
                if self._paused_since is not None:
                    self._paused_total += self._ended_at - self._paused_since
                    self._paused_since = None
                if self._cancel_requested:
                    final = RunState.CANCELLED
                elif self._process.returncode == 0:
                    final = RunState.SUCCEEDED
                else:
                    final = RunState.FAILED
                self._set_state(final)
            self._finished.set()

    def _handle_line(self, line):
        if self._on_output:
            self._on_output(line)
        updated = parse_progress(line, self._progress)
        if updated != self._progress:
            self._progress = updated
            if self._on_progress:
                self._on_progress(updated)


_HELP = "commands: p = pause, r = resume, c = cancel, s = status, h = help"


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        prog="kalibr_supervise",
        description="Run a Kalibr command with pause, resume and cancel. "
        "While it runs, type a command and press Enter. " + _HELP + ". "
        "Ctrl+C cancels; a second Ctrl+C kills immediately.",
    )
    parser.add_argument("--log", help="append the command's full output to this file")
    parser.add_argument("--quiet", action="store_true", help="do not echo the command's output")
    parser.add_argument("--cancel-timeout", type=float, default=10.0,
                        help="seconds to wait after SIGTERM before SIGKILL (default: 10)")
    parser.add_argument("command", nargs=argparse.REMAINDER,
                        help="the Kalibr command and its arguments (after --)")
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("no command given")

    def say(text):
        print("[supervisor] " + text, file=sys.stderr, flush=True)

    def echo(line):
        # Progress-bar redraws are summarized by the status command instead.
        if line.strip() and not _PROGRESS.search(line):
            print(line, flush=True)

    def report_progress(progress):
        if progress.done is None:  # phase or iteration changed, not a counter tick
            say("progress: " + progress.describe())

    supervisor = CalibrationSupervisor(
        command,
        log_path=args.log,
        on_state=lambda state: say("state: " + state.value),
        on_output=None if args.quiet else echo,
        on_progress=report_progress,
    )

    def status():
        progress = supervisor.progress
        say("{} | {} | {:.0f}s active".format(supervisor.state.value, progress.describe(), supervisor.active_seconds()))

    def commands():
        actions = {"p": supervisor.pause, "r": supervisor.resume, "s": status, "h": lambda: say(_HELP)}
        for line in sys.stdin:
            key = line.strip().lower()[:1]
            if not key:
                continue
            if key == "c":
                threading.Thread(target=supervisor.cancel, args=(args.cancel_timeout,), daemon=True).start()
                continue
            action = actions.get(key)
            if action is None:
                say(_HELP)
                continue
            try:
                action()
            except InvalidTransition as error:
                say(str(error))

    supervisor.start()
    say("started pid {} | {}".format(supervisor.pid, _HELP))
    if sys.stdin and sys.stdin.isatty():
        threading.Thread(target=commands, name="commands", daemon=True).start()

    # Installed explicitly so the run is cancelled, not orphaned, even when
    # SIGINT was inherited as ignored or another program sends SIGTERM.
    interrupts = []

    def on_signal(signum, frame):
        interrupts.append(signum)
        if len(interrupts) == 1:
            say("cancelling ({} again to kill)".format(signal.Signals(signum).name))
            threading.Thread(target=supervisor.cancel, args=(args.cancel_timeout,), daemon=True).start()
        else:
            supervisor._signal_group(signal.SIGKILL)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    while not supervisor.state.finished:
        supervisor.wait(0.2)
    status()
    if supervisor.state is RunState.SUCCEEDED:
        return 0
    return 130 if supervisor.state is RunState.CANCELLED else (supervisor.returncode or 1)


if __name__ == "__main__":
    raise SystemExit(main())
