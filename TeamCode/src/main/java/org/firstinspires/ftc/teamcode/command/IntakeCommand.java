package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 待机时 Intake 正转，Transit 在 LT 吸球和射击送料时运行。 */
public final class IntakeCommand extends CommandBase {
    private static final double IDLE_INTAKE_POWER = 0.7;

    private final IntakeSubsystem intake;
    private final Gamepad gamepad;
    private final BooleanSupplier canFeed;
    private boolean intakeEnabled;

    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad, BooleanSupplier canFeed) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.canFeed = canFeed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intakeEnabled = false;
    }

    @Override
    public void execute() {
        intakeEnabled = gamepad.left_trigger > ShootConstants.TRIGGER_THRESHOLD;
        boolean shootRequested = gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
        double intakePower = IDLE_INTAKE_POWER;
        double transitPower = 0.0;
        if (shootRequested) {
            boolean feeding = canFeed.getAsBoolean();
            intakePower = feeding ? ShootConstants.INTAKE_ACTIVE_POWER : 0.0;
            transitPower = feeding ? ShooterSubsystem.SHOT_TRANSIT_POWER : 0.0;
        } else if (gamepad.dpad_down || gamepad.left_bumper) {
            intakePower = ShootConstants.INTAKE_JAM_POWER;
        } else if (intakeEnabled) {
            intakePower = ShootConstants.INTAKE_ACTIVE_POWER;
            transitPower = ShootConstants.TRANSIT_FEED_POWER;
        }
        intake.setPowers(intakePower, transitPower);
    }

    @Override
    public void end(boolean interrupted) {
        intakeEnabled = false;
        intake.stop();
    }

    public boolean isIntakeEnabled() { return intakeEnabled; }
}
