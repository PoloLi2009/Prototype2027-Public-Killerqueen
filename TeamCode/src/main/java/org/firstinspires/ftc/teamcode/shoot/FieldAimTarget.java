package org.firstinspires.ftc.teamcode.shoot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.vision.CellTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.vision.VisionLocalization;

/** Uses Spinup's field-center pose and CELL targets when available, with KQ's original OD fallback. */
public final class FieldAimTarget {
    private final LocalizationSubsystem localization;
    private final LimelightVision camera;
    private final VisionLocalization fusion = new VisionLocalization();
    private final ShotTarget target = new ShotTarget();
    private boolean previousAllianceButton;
    private boolean targetChanged;
    private CellTarget cell;

    public FieldAimTarget(HardwareMap hardwareMap, LocalizationSubsystem localization) {
        this.localization = localization;
        camera = new LimelightVision(hardwareMap);
    }

    public void update(boolean allianceButton, boolean odometryChanged) {
        if (allianceButton && !previousAllianceButton) camera.toggleAlliance();
        previousAllianceButton = allianceButton;

        CellTarget previousCell = cell;
        cell = null;
        target.invalidate();
        camera.update();

        boolean validOdometry = localization.isFieldPoseValid();
        if (odometryChanged || !validOdometry) fusion.clear();

        if (validOdometry) {
            double x = localization.getXIn();
            double y = localization.getYIn();
            double heading = Math.toRadians(localization.getHeadingDeg());
            if (camera.isConfigured()) {
                Pose2d odometry = new Pose2d(DistanceUnit.INCH.toMeters(x),
                        DistanceUnit.INCH.toMeters(y), new Rotation2d(heading));
                fusion.update(System.nanoTime() / 1e9, odometry, camera.getNewFrame());
            }
            Pose2d pose = fusion.getEstimatedPosition();
            if (pose != null) {
                cell = CellTarget.forRobotX(camera.isBlueAlliance(), pose.getX() * 1000.0);
                if (cell != null) {
                    target.update(DistanceUnit.METER.toInches(pose.getX()),
                            DistanceUnit.METER.toInches(pose.getY()), pose.getRotation().getRadians(),
                            DistanceUnit.MM.toInches(cell.xMm), DistanceUnit.MM.toInches(cell.yMm));
                }
            } else {
                target.update(x, y, heading);
            }
        }

        targetChanged = odometryChanged || cell != previousCell;
    }

    public boolean hasValidAimingTarget() { return target.isValid(); }
    public double getYawErrorDegrees() { return target.getYawErrorDegrees(); }
    public boolean hasTargetChanged() { return targetChanged; }
    public String getAimSource() { return cell == null ? "Pinpoint" : "视觉融合 / " + cell.name(); }
    public String getCameraStatus() {
        return camera.isConfigured() ? (camera.isBlueAlliance() ? "蓝方" : "红方")
                + " / Pipeline " + camera.getExpectedPipeline() + " / " + camera.getStatus()
                : camera.getStatus();
    }
    public String getFusionStatus() { return fusion.getStatus(); }
    public Pose2d getFusedPose() { return fusion.getEstimatedPosition(); }
    public void stop() { camera.stop(); }
}
