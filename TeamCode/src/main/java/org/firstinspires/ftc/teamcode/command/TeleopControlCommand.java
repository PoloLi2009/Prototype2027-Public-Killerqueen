package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.shoot.AutoAimController;
import org.firstinspires.ftc.teamcode.shoot.FieldAimTarget;
import org.firstinspires.ftc.teamcode.shoot.ShootConstants;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 双摇杆驾驶、B 底盘自瞄、飞轮常转、RT 达速开挡板并送料。 */
public final class TeleopControlCommand extends CommandBase {
    private final Gamepad gamepad;
    private final Gamepad operatorGamepad;
    public static final double PINPOINT_TRIM_STEP_IN = 1.0;
    private final PinpointTrimButtons trimButtons = new PinpointTrimButtons();
    private String trimStatus = "未调整";
    private final DoubleSupplier time;
    private final LocalizationSubsystem localization;
    private final BlockerSubsystem blocker;
    private final ShooterSubsystem shooter;
    private final DriveCommand driveCommand;
    private final ShooterCommand shooterCommand;
    private final IntakeCommand intakeCommand;
    private final ChassisAimCommand aimCommand;
    private final FieldAimTarget aimTarget;
    private final AutoAimController autoAim;
    private final ElapsedTime blockerOpenTimer = new ElapsedTime();

    private double lastTime, deltaSeconds;
    private boolean previousB;
    private boolean previousLeftStickButton;
    private boolean previousOptions;
    private boolean aiming;

    public TeleopControlCommand(Gamepad gamepad, Gamepad operatorGamepad, DoubleSupplier time,
            LocalizationSubsystem localization, DriveSubsystem drive,
            IntakeSubsystem intake, ShooterSubsystem shooter,
            TurretSubsystem turret, BlockerSubsystem blocker, FieldAimTarget aimTarget) {
        this.gamepad = gamepad;
        this.operatorGamepad = operatorGamepad;
        this.time = time;
        this.localization = localization;
        this.blocker = blocker;
        this.shooter = shooter;
        driveCommand = new DriveCommand(drive, localization,
                () -> -gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x);
        shooterCommand = new ShooterCommand(shooter, this::isShotRequested, () -> deltaSeconds);
        intakeCommand = new IntakeCommand(intake, gamepad, () -> blocker.isOpen()
                && blockerOpenTimer.seconds() >= ShooterSubsystem.TRANSIT_DELAY_SECONDS);
        this.aimTarget = aimTarget;
        autoAim = new AutoAimController();
        aimCommand = new ChassisAimCommand(drive, localization, aimTarget, autoAim, gamepad);
        addRequirements(localization, turret, blocker);
        for (Command child : new Command[] {driveCommand, shooterCommand, intakeCommand, aimCommand}) {
            addRequirements(child.getRequirements().toArray(new Subsystem[0]));
        }
    }

    @Override
    public void initialize() {
        lastTime = time.getAsDouble();
        previousB = gamepad.b;
        previousLeftStickButton = gamepad.left_stick_button;
        previousOptions = gamepad.options;
        trimButtons.update(operatorGamepad.dpad_up, operatorGamepad.dpad_down,
                operatorGamepad.dpad_left, operatorGamepad.dpad_right);
        trimStatus = "未调整";
        aiming = false;
        localization.update();
        localization.resetDriverHeading(); // OT: drive.resetDriverHeading() at START
        blocker.setOpen(false);
        driveCommand.initialize();
        intakeCommand.initialize();
    }

    @Override
    public void execute() {
        double now = time.getAsDouble();
        deltaSeconds = Math.max(now - lastTime, 1e-4);
        lastTime = now;
        localization.update();
        if ("等待 Pinpoint 恢复".equals(trimStatus) && localization.isFieldPoseValid()) {
            trimStatus = "已调整";
        }
        boolean resetPressed = gamepad.options && !previousOptions;
        boolean odometryChanged = resetPressed && localization.resetToFieldCenter();
        previousOptions = gamepad.options;
        trimButtons.update(operatorGamepad.dpad_up, operatorGamepad.dpad_down,
                operatorGamepad.dpad_left, operatorGamepad.dpad_right);
        int dx = trimButtons.getXDirection(), dy = trimButtons.getYDirection();
        if (!resetPressed && (dx != 0 || dy != 0)) {
            boolean accepted = localization.adjustFieldPosition(
                    dx * PINPOINT_TRIM_STEP_IN, dy * PINPOINT_TRIM_STEP_IN);
            odometryChanged = accepted;
            trimStatus = !accepted ? "未调整：定位无效" : localization.isFieldPoseValid()
                    ? "已调整" : "等待 Pinpoint 恢复";
        }
        aimTarget.update(gamepad.y, odometryChanged);
        // OT: 左摇杆按下重新设定驾驶员前方。
        if (gamepad.left_stick_button && !previousLeftStickButton) localization.resetDriverHeading();
        previousLeftStickButton = gamepad.left_stick_button;
        updateAimButton();
        shooterCommand.execute();
        updateBlocker();
        blocker.flushOutput(); // 每周期写出，Dashboard 改挡板位置立即生效
        intakeCommand.execute();
        if (aiming) {
            aimCommand.execute();
        } else {
            driveCommand.execute();
        }
    }

    public boolean isIntakeEnabled() { return intakeCommand.isIntakeEnabled(); }

    public String getTrimStatus() { return trimStatus; }

    /**
     * 按住 RT 且飞轮达速（ShooterSubsystem.isReady，OT 规则，不看自瞄）时打开挡板 blockerServo。
     * 打开后在 RT 松开前保持打开（同 OT：开闸后掉速不中断），松开 RT 立即关闭。
     */
    private void updateBlocker() {
        if (!isShotRequested()) {
            if (blocker.isOpen()) blocker.setOpen(false);
        } else if (!blocker.isOpen() && shooter.isReady()) {
            blocker.setOpen(true);
            blockerOpenTimer.reset();
        }
    }

    private boolean isShotRequested() {
        return gamepad.right_trigger > ShootConstants.TRIGGER_THRESHOLD;
    }

    private void updateAimButton() {
        boolean pressed = gamepad.b;
        if (pressed && !previousB) {
            aimCommand.initialize();
            aiming = true;
        } else if (!pressed && aiming) {
            aimCommand.end(false);
            aiming = false;
        }
        previousB = pressed;
    }

    @Override
    public void end(boolean interrupted) {
        if (aiming) aimCommand.end(interrupted);
        aiming = false;
        driveCommand.end(interrupted);
        intakeCommand.end(interrupted);
        shooterCommand.end(interrupted);
        blocker.setOpen(false);
    }

    public DriveCommand getDriveCommand() {
        return driveCommand;
    }

    public boolean isAiming() {
        return aiming;
    }

    public AutoAimController getAutoAim() {
        return autoAim;
    }
}
