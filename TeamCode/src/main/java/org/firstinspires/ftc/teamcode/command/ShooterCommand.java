package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/** 在待机和射击转速之间切换，并每周期更新飞轮闭环。 */
public final class ShooterCommand extends CommandBase {
    private static final double STANDBY_RPM = 2000.0;

    private final ShooterSubsystem shooter;
    private final BooleanSupplier shotRequested;
    private final DoubleSupplier deltaSeconds;

    public ShooterCommand(ShooterSubsystem shooter, BooleanSupplier shotRequested,
                          DoubleSupplier deltaSeconds) {
        this.shooter = shooter;
        this.shotRequested = shotRequested;
        this.deltaSeconds = deltaSeconds;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double targetRpm = shotRequested.getAsBoolean() ? shooter.getShotRpm() : STANDBY_RPM;
        shooter.setTargetRpm(targetRpm);
        shooter.update(deltaSeconds.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.update(1e-4);
    }
}
