import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/launcher/backend"))
from profiles import Inspector, command_for
from runtime import Supervisor


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.inspector = Inspector(ROOT)

    def test_builtin_profiles_are_discoverable_without_install(self):
        result = self.inspector.profiles({})
        self.assertEqual(
            {p["name"] for p in result["profiles"]}, {"dm_arm_gray", "dm_arm_white"}
        )
        self.assertTrue(result["profile_file"].endswith("robot_profiles.yaml"))

    def test_inspection_is_readonly_and_exposes_hardware_state(self):
        result = self.inspector.inspect({"profile": "dm_arm_gray"})
        self.assertTrue(result["write_enabled"])
        self.assertEqual(result["devices"], ["/dev/ttyACM0"])
        self.assertTrue(Path(result["resources"]["core"]).is_file())
        self.assertFalse(result["available"]["terminal"])

    def test_missing_profile_is_actionable(self):
        with self.assertRaises(ValueError):
            self.inspector.inspect({"profile": "unknown"})

    def test_argument_building_keeps_shell_text_literal(self):
        config = {
            "profile": "robot; touch /tmp/unsafe",
            "profile_file": "/tmp/a b.yaml",
            "serial_port": "/dev/ttyUSB0",
            "resource_paths": "/tmp/one:/tmp/two",
        }
        args, env = command_for("terminal", config)
        self.assertIn("robot; touch /tmp/unsafe", args)
        self.assertEqual(env["SERIAL_ARM_RESOURCE_PATH"], "/tmp/one:/tmp/two")
        args, _ = command_for("model", config)
        self.assertNotIn("serial_port:=/dev/ttyUSB0", args)
        self.assertIn("profile_file:=/tmp/a b.yaml", args)

    def test_each_ros_mode_targets_an_installed_launch_source(self):
        launch_dir = ROOT / "src/serial_arm/bringup/ros2_control/launch"
        for mode, name in [
            ("model", "display.launch.py"),
            ("hardware", "hardware.launch.py"),
            ("moveit", "moveit.launch.py"),
        ]:
            args, _ = command_for(mode, {"profile": "dm_arm_gray"})
            self.assertEqual(args[3], name)
            self.assertTrue((launch_dir / args[3]).is_file())

    def test_invalid_mode_and_baudrate_rejected(self):
        with self.assertRaises(ValueError):
            command_for("shell", {"profile": "x"})
        with self.assertRaises(ValueError):
            command_for("terminal", {"profile": "x", "baudrate": "bad"})


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.events = []
        self.manager = Supervisor(self.events.append, Path(self.tmp.name))
        self.addCleanup(self.manager.close)

    def wait(self, test):
        deadline = time.time() + 4
        while time.time() < deadline:
            if test():
                return
            time.sleep(0.02)
        self.fail("runtime timeout")

    def test_pty_is_interactive_and_input_reaches_process(self):
        self.manager.start(
            [
                sys.executable,
                "-u",
                "-c",
                'import sys; print("READY",sys.stdin.isatty()); print(input())',
            ],
            dict(os.environ),
            [],
            "terminal",
        )
        self.wait(
            lambda: "READY True" in "".join(e.get("data", "") for e in self.events)
        )
        self.manager.input("hello\r")
        self.wait(lambda: self.manager.status()["state"] == "exited")
        self.assertIn("hello", "".join(e.get("data", "") for e in self.events))

    def test_device_lock_prevents_second_launcher_and_is_released(self):
        second = Supervisor(lambda e: None, Path(self.tmp.name))
        self.addCleanup(second.close)
        self.manager.start(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            dict(os.environ),
            ["/dev/test-device"],
            "hardware",
        )
        with self.assertRaises(ValueError):
            second.start(
                [sys.executable, "-c", "pass"],
                dict(os.environ),
                ["/dev/test-device"],
                "hardware",
            )
        self.manager.stop()
        self.wait(lambda: self.manager.status()["state"] == "exited")
        second.start(
            [sys.executable, "-c", "pass"],
            dict(os.environ),
            ["/dev/test-device"],
            "hardware",
        )

    def test_only_one_live_session_and_start_failure_releases_locks(self):
        with self.assertRaises(OSError):
            self.manager.start(
                ["/does/not/exist"], dict(os.environ), ["/dev/test"], "hardware"
            )
        self.manager.start(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            dict(os.environ),
            ["/dev/test"],
            "hardware",
        )
        with self.assertRaises(ValueError):
            self.manager.start(
                [sys.executable, "-c", "pass"], dict(os.environ), [], "terminal"
            )


if __name__ == "__main__":
    unittest.main()
