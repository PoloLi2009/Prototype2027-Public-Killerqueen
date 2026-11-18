package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

abstract class ServoCalibrationOpMode extends OpMode {
    private static final double START_POSITION = 0.50;
    private static final double POSITION_STEP = 0.01;

    private final String servoName;
    private Servo servo;
    private double position;
    private boolean previousUp;
    private boolean previousDown;

    ServoCalibrationOpMode(String servoName) {
        this.servoName = servoName;
    }

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, servoName);
        position = START_POSITION;
        telemetry.addLine("按 START 将 " + servoName + " 设到 0.50");
        telemetry.addLine("D-pad 上/下每按一次调整 +0.01 / -0.01");
        telemetry.update();
    }

    @Override
    public void start() {
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        servo.setPosition(position);
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up && !previousUp && !down) {
            setPosition(position + POSITION_STEP);
        } else if (down && !previousDown && !up) {
            setPosition(position - POSITION_STEP);
        }
        previousUp = up;
        previousDown = down;

        telemetry.addData(servoName + " 指令位置", "%.2f", position);
        telemetry.addLine("对齐机械零点后，记下上面的数值");
        telemetry.update();
    }

    private void setPosition(double requestedPosition) {
        position = Range.clip(requestedPosition, 0.0, 1.0);
        servo.setPosition(position);
    }
}
