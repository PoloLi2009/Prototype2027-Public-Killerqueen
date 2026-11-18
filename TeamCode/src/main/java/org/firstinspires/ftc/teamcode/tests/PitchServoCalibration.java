package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NameConstants;

@TeleOp(name = "Pitch Servo Calibration", group = "Test")
public final class PitchServoCalibration extends ServoCalibrationOpMode {
    public PitchServoCalibration() {
        super(NameConstants.PITCH_SERVO);
    }
}
