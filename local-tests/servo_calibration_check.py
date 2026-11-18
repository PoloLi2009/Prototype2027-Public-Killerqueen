#!/usr/bin/env python3
"""Check fixed turret output and servo calibration against FTC hardware substitutes."""
import os
from pathlib import Path
import runpy
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA_ROOT = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
REPLAY_STUBS = runpy.run_path(str(ROOT / "local-tests/run_replay.py"))["STUBS"]
STUBS = {
    path: REPLAY_STUBS[path]
    for path in (
        "com/qualcomm/robotcore/hardware/HardwareMap.java",
        "com/qualcomm/robotcore/hardware/Servo.java",
        "com/qualcomm/robotcore/hardware/Gamepad.java",
        "com/qualcomm/robotcore/eventloop/opmode/OpMode.java",
        "com/qualcomm/robotcore/eventloop/opmode/TeleOp.java",
    )
}
STUBS.update({
    "com/arcrobotics/ftclib/command/SubsystemBase.java":
        "package com.arcrobotics.ftclib.command; public class SubsystemBase {}",
    "com/qualcomm/robotcore/util/Range.java": """
        package com.qualcomm.robotcore.util;
        public class Range {
            public static double clip(double value, double min, double max) {
                return Math.max(min, Math.min(max, value));
            }
        }""",
    "ServoCalibrationCheck.java": """
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import org.firstinspires.ftc.teamcode.NameConstants;
        import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;
        import org.firstinspires.ftc.teamcode.tests.FlowerServoCalibration;
        import org.firstinspires.ftc.teamcode.tests.PitchServoCalibration;

        public class ServoCalibrationCheck {
            static void near(double actual, double expected, String label) {
                if (!(Math.abs(actual - expected) < 1e-8))
                    throw new AssertionError(label + ": " + actual + " != " + expected);
            }

            static void press(OpMode calibration, boolean up, boolean down) {
                calibration.gamepad1.dpad_up = up;
                calibration.gamepad1.dpad_down = down;
                calibration.loop();
                calibration.gamepad1.dpad_up = false;
                calibration.gamepad1.dpad_down = false;
                calibration.loop();
            }

            public static void main(String[] args) {
                HardwareMap turretHardware = new HardwareMap();
                Servo left = new Servo(), right = new Servo();
                turretHardware.devices.put(NameConstants.TURRET_LEFT, left);
                turretHardware.devices.put(NameConstants.TURRET_RIGHT, right);
                TurretSubsystem turret = new TurretSubsystem(turretHardware);
                near(left.position, 0.50, "left INIT");
                near(right.position, 0.50, "right INIT");
                left.setPosition(0.1);
                right.setPosition(0.9);
                turret.center();
                near(left.position, 0.50, "left START");
                near(right.position, 0.50, "right START");

                checkCalibration(new PitchServoCalibration(), "pitchServo");
                checkCalibration(new FlowerServoCalibration(), "flowerServo");
                System.out.println("PASS: turret center; pitch and flower calibration, steps, limits and telemetry");
            }

            static void checkCalibration(OpMode calibration, String servoName) {
                Servo servo = new Servo();
                servo.setPosition(0.25);
                calibration.hardwareMap.devices.put(servoName, servo);
                calibration.init();
                near(servo.position, 0.25, "INIT leaves servo position alone");
                calibration.gamepad1.dpad_up = true;
                calibration.start();
                calibration.loop();
                near(servo.position, 0.50, "START sets midpoint and ignores a held button");
                press(calibration, false, false);
                press(calibration, true, false);
                near(servo.position, 0.51, "up increases by 0.01");
                calibration.gamepad1.dpad_down = true;
                calibration.loop();
                calibration.loop();
                near(servo.position, 0.50, "held down moves only once");
                press(calibration, false, false);
                press(calibration, true, true);
                near(servo.position, 0.50, "opposite buttons cancel");
                for (int i = 0; i < 60; i++) press(calibration, true, false);
                near(servo.position, 1.00, "upper clamp");
                for (int i = 0; i < 110; i++) press(calibration, false, true);
                near(servo.position, 0.00, "lower clamp");
                if (!"0.00".equals(calibration.telemetry.values.get(servoName + " 指令位置")))
                    throw new AssertionError("calibration displays the commanded position");
            }
        }""",
})

with tempfile.TemporaryDirectory(prefix="servo-calibration-") as temp:
    root = Path(temp)
    for name, source in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding="utf-8")
    sources = [JAVA_ROOT / name for name in (
        "NameConstants.java", "subsystem/TurretSubsystem.java", "tests/ServoCalibrationOpMode.java",
        "tests/PitchServoCalibration.java", "tests/FlowerServoCalibration.java"
    )]
    java_bin = Path(os.environ.get("JAVA_HOME", "/usr")) / "bin"
    subprocess.run([str(java_bin / "javac"), "-encoding", "UTF-8", "-d", temp,
                    *map(str, root.rglob("*.java")), *map(str, sources)], check=True)
    subprocess.run([str(java_bin / "java"), "-cp", temp, "ServoCalibrationCheck"], check=True)
