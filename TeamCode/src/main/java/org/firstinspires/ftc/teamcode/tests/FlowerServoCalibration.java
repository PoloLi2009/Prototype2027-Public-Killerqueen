package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NameConstants;

@TeleOp(name = "Flower Servo Calibration", group = "Test")
public final class FlowerServoCalibration extends ServoCalibrationOpMode {
    public FlowerServoCalibration() {
        super(NameConstants.FLOWER_SERVO);
    }
}
