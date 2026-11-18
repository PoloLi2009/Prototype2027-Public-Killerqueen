package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.command.TeleopControlCommand;
import org.firstinspires.ftc.teamcode.shoot.FieldAimTarget;
import org.firstinspires.ftc.teamcode.subsystem.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;

/** 2026 Biobuzz 比赛手动控制入口。 */
@Config
@TeleOp(name = "Solo", group = "Competition")
public class Teleop extends OpMode {
    private static final double WALL_SHOT_RPM = 3450.0;
    private static final double PITCH_SERVO_POSITION = 0.52;
    private static final double FLOWER_LOWERED_POSITION = 0.20;
    private static final double FLOWER_RAISED_POSITION = 1.00;
    public static double WALL_SHOT_KP = 0.000360000;
    private static final double WALL_SHOT_KI = 0.0;
    public static double WALL_SHOT_KD = 0.000010000;

    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private BlockerSubsystem blocker;
    private Servo pitchServo;
    private Servo flowerServo;
    private boolean flowerRaised;
    private boolean previousRightStickButton;

    private FieldAimTarget aimTarget;
    private TeleopControlCommand controls;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        turret = new TurretSubsystem(hardwareMap);
        pitchServo = hardwareMap.get(Servo.class, NameConstants.PITCH_SERVO);
        flowerServo = hardwareMap.get(Servo.class, NameConstants.FLOWER_SERVO);
        pitchServo.setPosition(PITCH_SERVO_POSITION);
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, () -> WALL_SHOT_RPM,
                () -> WALL_SHOT_KP, () -> WALL_SHOT_KI, () -> WALL_SHOT_KD);
        shooter.setShotMode(ShooterSubsystem.ShotMode.NEAR);
        blocker = new BlockerSubsystem(hardwareMap);

        aimTarget = new FieldAimTarget(hardwareMap, localization);
        controls = new TeleopControlCommand(gamepad1, gamepad2, this::getRuntime,
                localization, drive, intake, shooter, turret, blocker, aimTarget);

        telemetry.addLine(localization.isPinpointReady() ? "Pinpoint: 已启用" : "Pinpoint: 未找到");
        addControlHelp();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // 直接读取 Pinpoint 当前位姿，自动到手动切换时不重置设备。
        localization.update();
        aimTarget.update(gamepad1.y, false);

        telemetry.addData("初始坐标来源", "Pinpoint 当前位姿");
        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        telemetry.addData("当前坐标有效", localization.isFieldPoseValid());
        if (localization.isFieldPoseValid()) {
            telemetry.addData("当前 X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("当前 Heading (°)", "%.1f", localization.getHeadingDeg());
        } else {
            telemetry.addLine("等待 Pinpoint READY，尚未读取有效坐标");
        }
        addVisionTelemetry();
        telemetry.update();
    }

    /** 主操负责驾驶机构，副操方向键微调 Pinpoint 坐标。 */
    private void addControlHelp() {
        telemetry.addLine("内置 IMU 无头平移（START 时车头为前方，按下左摇杆重设），右摇杆转向");
        telemetry.addLine("按 Option 将 Pinpoint 坐标设为 (72, 72 in, 180°)");
        telemetry.addLine("副操方向键：上/下 X ±1 in，右/左 Y ±1 in（每按一次）");
        telemetry.addLine("按住 B 底盘自瞄，左摇杆仍可平移。Y 切换视觉红蓝方");
        telemetry.addLine("主操按下右摇杆切换 Flower 铲子抬起 / 放下");
        telemetry.addData("云台锁定位置", "%.2f", TurretSubsystem.CENTER_POSITION);
        telemetry.addData("贴墙射击", "按住 RT，%.0f RPM，角度舵机 %.2f",
                shooter.getShotRpm(), shooter.getServoPosition());
    }

    @Override
    public void start() {
        turret.center();
        pitchServo.setPosition(PITCH_SERVO_POSITION);
        flowerRaised = true;
        flowerServo.setPosition(FLOWER_RAISED_POSITION);
        previousRightStickButton = gamepad1.right_stick_button;
        CommandScheduler.getInstance().schedule(controls);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        boolean rightStickButton = gamepad1.right_stick_button;
        if (rightStickButton && !previousRightStickButton) {
            flowerRaised = !flowerRaised;
            flowerServo.setPosition(flowerRaised ? FLOWER_RAISED_POSITION : FLOWER_LOWERED_POSITION);
        }
        previousRightStickButton = rightStickButton;
        updateTelemetry();
    }

    private void updateTelemetry() {
        double yawDeg = localization.getYawDegrees();
        telemetry.addData("机器人场地 Yaw (°)", "%.1f", yawDeg);
        double driverHeading = localization.getManualDriveHeadingRadians();
        telemetry.addData("底盘锁定", controls.getDriveCommand().isHolding() ? "电机刹车锁死" : "否");
        telemetry.addData("底盘模式", Double.isFinite(driverHeading)
                ? String.format(java.util.Locale.US, "IMU 无头（相对前方 %.1f°）", Math.toDegrees(driverHeading))
                : "IMU 航向无效，底盘停止");
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                drive.getFrontLeftPower(), drive.getBackLeftPower(),
                drive.getFrontRightPower(), drive.getBackRightPower());

        telemetry.addData("Pinpoint 状态", localization.getPinpointStatus());
        telemetry.addData("副操坐标微调", controls.getTrimStatus());
        if (localization.isPinpointReady()) {
            telemetry.addData("Pinpoint X/Y (in)", "(%.2f, %.2f)",
                    localization.getXIn(), localization.getYIn());
            telemetry.addData("Pinpoint Heading (°)", "%.1f", localization.getHeadingDeg());
            telemetry.addData("Pinpoint - IMU 差值 (°)", "%.1f",
                    AngleUnit.normalizeDegrees(localization.getHeadingDeg() - localization.getImuYawDegrees()));
        }

        telemetry.addData("云台锁定位置", "%.2f", TurretSubsystem.CENTER_POSITION);
        telemetry.addData("挡板", "%s (%.2f)", blocker.isOpen() ? "开" : "关", blocker.getPosition());
        addVisionTelemetry();
        telemetry.addData("底盘转向", controls.isAiming() ? aimTarget.getAimSource() : "右摇杆");
        if (controls.isAiming()) {
            telemetry.addData("瞄点有效", aimTarget.hasValidAimingTarget());
            telemetry.addData("瞄准误差 (°)", "%.1f", aimTarget.getYawErrorDegrees());
            telemetry.addData("底盘已对准", controls.getAutoAim().isLocked());
        }
        telemetry.addData("飞轮达速", shooter.isReady() ? "是" : "否");

        telemetry.addData("Shooter 目标 RPM", "%.1f", shooter.getTargetRpm());
        telemetry.addData("Shooter 实际 RPM", "%.0f", shooter.getFilteredRpm());
        telemetry.addData("飞轮编码器", shooter.getEncoderName());
        telemetry.addData("Shooter 误差 RPM", "%.0f", shooter.getError());
        telemetry.addData("Shooter P / I / D", "%.9f / %.9f / %.9f",
                WALL_SHOT_KP, WALL_SHOT_KI, WALL_SHOT_KD);
        telemetry.addData("Shooter 功率", "%.3f", shooter.getPower());
        telemetry.addData("贴墙射击角度舵机", "%.3f", shooter.getServoPosition());
        telemetry.addData("pitchServo 固定位置", "%.2f", PITCH_SERVO_POSITION);
        telemetry.addData("Flower 铲子", flowerRaised ? "抬起" : "放下");
        if (!shooter.getControlFault().isEmpty()) telemetry.addData("飞轮控制故障", shooter.getControlFault());


        telemetry.addData("Intake 功率", "%.2f", intake.getIntakePower());
        telemetry.addData("Transit 功率", "%.2f", intake.getTransitPower());
        telemetry.addData("LT 吸球请求", controls.isIntakeEnabled() ? "开" : "关");

        telemetry.update();
    }

    private void addVisionTelemetry() {
        telemetry.addData("Limelight", aimTarget.getCameraStatus());
        telemetry.addData("视觉融合", aimTarget.getFusionStatus());
        Pose2d pose = aimTarget.getFusedPose();
        if (pose != null) {
            telemetry.addData("融合位姿 X/Y (m), 航向 (°)", "%.2f / %.2f / %.1f",
                    pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        }
    }

    @Override
    public void stop() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        if (scheduler.isScheduled(controls)) {
            scheduler.cancelAll();
        } else {
            // INIT 后直接 STOP 时组合命令尚未启动。
            drive.stop();
            intake.stop();
            shooter.stop();
            shooter.update(1e-4);
            blocker.setOpen(false);
        }
        scheduler.reset();
        if (aimTarget != null) aimTarget.stop();
    }
}
