package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.NameConstants;

/** 比赛云台的双舵机始终使用中位。 */
public final class TurretSubsystem extends SubsystemBase {
    public static final double CENTER_POSITION = 0.5;

    private final Servo leftServo;
    private final Servo rightServo;

    public TurretSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, NameConstants.TURRET_LEFT);
        rightServo = hardwareMap.get(Servo.class, NameConstants.TURRET_RIGHT);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
        center();
    }

    public void center() {
        leftServo.setPosition(CENTER_POSITION);
        rightServo.setPosition(CENTER_POSITION);
    }
}
