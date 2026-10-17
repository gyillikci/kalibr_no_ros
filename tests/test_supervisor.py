import os
import shutil
import sys
import tempfile
import textwrap
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "bag_io" / "src"), str(ROOT / "core" / "src")]

from kalibr_no_ros.supervisor import (
    CalibrationSupervisor,
    InvalidTransition,
    Progress,
    RunState,
    parse_progress,
)

# Stands in for Kalibr: worker processes in the same group, '\r' progress
# redraws and optimizer iteration lines.
FAKE_KALIBR = textwrap.dedent(
    """
    import multiprocessing, signal, sys, time

    def worker(path):
        while True:
            with open(path, "a") as fh:
                fh.write("x")
            time.sleep(0.02)

    if __name__ == "__main__":
        mode, ticks = sys.argv[1], sys.argv[2]
        if mode == "ignore-term":
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print("Extracting calibration target corners", flush=True)
        workers = [multiprocessing.Process(target=worker, args=(ticks,), daemon=True) for _ in range(2)]
        for w in workers:
            w.start()
        steps = 3 if mode in ("ok", "fail") else 1000
        for i in range(1, steps + 1):
            sys.stdout.write("\\r  Progress {} / {} \\t Time remaining: 0s".format(i, steps))
            sys.stdout.flush()
            time.sleep(0.05)
        print("\\nBuilding the problem", flush=True)
        print("Optimization problem initialized with 10 design variables", flush=True)
        print("[1]: J: 5.0, dJ: 1.0", flush=True)
        sys.exit(3 if mode == "fail" else 0)
    """
)


def group_members(pgid):
    """Return {pid: state letter} for live processes in a process group."""
    members = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            stat = Path("/proc", entry, "stat").read_text()
        except OSError:
            continue
        fields = stat[stat.rindex(")") + 2:].split()
        if int(fields[2]) == pgid and fields[0] != "Z":
            members[int(entry)] = fields[0]
    return members


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.directory, ignore_errors=True)
        self.script = self.directory / "fake_kalibr.py"
        self.script.write_text(FAKE_KALIBR)
        self.ticks = self.directory / "ticks"
        self.ticks.touch()
        self.states = []

    def supervisor(self, mode, **kwargs):
        return CalibrationSupervisor(
            [sys.executable, str(self.script), mode, str(self.ticks)],
            on_state=self.states.append,
            **kwargs,
        )

    def wait_for_workers(self, run):
        self.assertTrue(wait_for(lambda: len(group_members(run.pid)) >= 3), "workers did not start")

    def test_success_reports_phases_and_states(self):
        log = self.directory / "run.log"
        run = self.supervisor("ok", log_path=log).start()
        self.assertEqual(run.wait(30), RunState.SUCCEEDED)
        self.assertEqual(self.states, [RunState.RUNNING, RunState.SUCCEEDED])
        self.assertEqual(run.returncode, 0)
        self.assertEqual(run.progress, Progress("optimizing", None, None, 1))
        self.assertIn("Building the problem", log.read_text())
        self.assertEqual(group_members(run.pid), {})

    def test_nonzero_exit_is_failed(self):
        run = self.supervisor("fail").start()
        self.assertEqual(run.wait(30), RunState.FAILED)
        self.assertEqual(run.returncode, 3)

    def test_pause_stops_every_process_and_resume_continues(self):
        run = self.supervisor("long").start()
        self.wait_for_workers(run)
        run.pause()
        self.assertTrue(wait_for(lambda: set(group_members(run.pid).values()) == {"T"}))
        size = self.ticks.stat().st_size
        time.sleep(0.3)
        self.assertEqual(self.ticks.stat().st_size, size, "workers kept running while paused")

        run.resume()
        self.assertTrue(wait_for(lambda: "T" not in group_members(run.pid).values()))
        self.assertTrue(wait_for(lambda: self.ticks.stat().st_size > size))

        run.cancel(timeout=5)
        self.assertEqual(run.wait(10), RunState.CANCELLED)
        self.assertEqual(
            self.states,
            [RunState.RUNNING, RunState.PAUSED, RunState.RUNNING, RunState.CANCELLING, RunState.CANCELLED],
        )
        self.assertEqual(group_members(run.pid), {})

    def test_cancel_while_paused(self):
        run = self.supervisor("long").start()
        self.wait_for_workers(run)
        run.pause()
        run.cancel(timeout=5)
        self.assertEqual(run.wait(10), RunState.CANCELLED)
        self.assertEqual(group_members(run.pid), {})

    def test_cancel_escalates_to_sigkill(self):
        run = self.supervisor("ignore-term").start()
        self.wait_for_workers(run)
        started = time.monotonic()
        run.cancel(timeout=0.5)
        self.assertEqual(run.wait(10), RunState.CANCELLED)
        self.assertLess(time.monotonic() - started, 5)
        self.assertEqual(group_members(run.pid), {})

    def test_invalid_transitions_raise(self):
        run = self.supervisor("long")
        with self.assertRaises(InvalidTransition):
            run.pause()
        run.start()
        with self.assertRaises(InvalidTransition):
            run.resume()
        with self.assertRaises(InvalidTransition):
            run.start()
        run.cancel(timeout=5)
        run.wait(10)
        with self.assertRaises(InvalidTransition):
            run.pause()
        run.cancel()  # cancelling a finished run is a no-op

    def test_active_seconds_excludes_pauses(self):
        run = self.supervisor("long").start()
        self.wait_for_workers(run)
        run.pause()
        paused_at = run.active_seconds()
        time.sleep(0.5)
        self.assertAlmostEqual(run.active_seconds(), paused_at, places=3)
        run.cancel(timeout=5)
        run.wait(10)


class ParseProgressTest(unittest.TestCase):
    def test_kalibr_output_lines(self):
        progress = Progress()
        for line, expected in [
            ("Extracting calibration target corners", Progress("extracting corners")),
            ("  Progress 33074 / 40799 \t Time remaining: 11s", Progress("extracting corners", 33074, 40799)),
            ("Building the problem", Progress("building problem")),
            ("Optimization problem initialized with 32664 design variables and 504597 error terms",
             Progress("optimizing")),
            ("[2]: J: 121574, dJ: 122533, deltaX: 0.0452298, LM - lambda:3.33333 mu:2",
             Progress("optimizing", iteration=2)),
            ("After Optimization (Results)", Progress("writing results")),
            ("Generating result report...", Progress("writing report")),
            ("unrelated output", Progress("writing report")),
        ]:
            progress = parse_progress(line, progress)
            self.assertEqual(progress, expected, line)


if __name__ == "__main__":
    unittest.main()
