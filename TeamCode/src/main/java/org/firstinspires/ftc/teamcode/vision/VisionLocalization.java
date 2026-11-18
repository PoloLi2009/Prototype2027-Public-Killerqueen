package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.vision.fusion.PinpointEkf;
import org.firstinspires.ftc.teamcode.vision.fusion.PoseHistory;

/** Spinup's delayed MT1/Pinpoint fusion, using a camera calibrated in the fixed chassis frame. */
public final class VisionLocalization {
    private static final double MAX_FRAME_GAP_SECONDS = 0.20;
    private static final double MAX_TILT_CHANGE_DEGREES = 2.0;
    private static final double STABLE_TILT_SECONDS = 0.15;

    private final PoseHistory history = new PoseHistory();
    private final PinpointEkf ekf = new PinpointEkf();
    private String status = "等待 Pinpoint 和视觉";
    private double lastTilt = Double.NaN;
    private double stableSince;
    private double lastCapture;
    private int lastPipeline = -1;

    public void clear() {
        history.clear();
        ekf.reset();
        lastTilt = Double.NaN;
        lastPipeline = -1;
        status = "等待 Pinpoint 和视觉";
    }

    public void update(double now, Pose2d odometry, LimelightVision.Frame frame) {
        history.add(now, odometry);
        ekf.updateWithTime(now, odometry);
        if (frame == null) return;

        double capture = frame.approximateCaptureNanos / 1e9;
        Pose2d odometryAtCapture = history.sample(capture);
        if (odometryAtCapture == null) {
            status = "拍摄时刻超出 Pinpoint 历史";
            return;
        }

        Position p = frame.rawPose.getPosition().toUnit(DistanceUnit.METER);
        HiveFlatPoseSolver.Result result = HiveFlatPoseSolver.solve(p.x, p.y, p.z,
                frame.rawPose.getOrientation().getYaw(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getRoll(AngleUnit.RADIANS));
        if (result == null) {
            lastTilt = Double.NaN;
            status = "HIVE 几何或地面高度无效";
            return;
        }

        double tilt = result.hiveTiltDegrees;
        if (frame.pipeline != lastPipeline || !Double.isFinite(lastTilt)
                || capture - lastCapture > MAX_FRAME_GAP_SECONDS
                || Math.abs(tilt - lastTilt) > MAX_TILT_CHANGE_DEGREES) {
            stableSince = capture;
        }
        lastPipeline = frame.pipeline;
        lastCapture = capture;
        lastTilt = tilt;
        if (capture - stableSince < STABLE_TILT_SECONDS) {
            status = "等待 HIVE 倾角稳定";
            return;
        }

        double distance = frame.averageDistanceMeters;
        double xyStd = VisionConstants.minimumVisionXyStdMeters
                + VisionConstants.visionDistanceNoiseCoefficient * distance * distance / Math.sqrt(frame.tagCount);
        double headingStd = Math.toRadians(VisionConstants.visionHeadingStdDegrees);
        ekf.addVisionMeasurement(result.chassisPose, capture, xyStd, headingStd, odometryAtCapture);
        status = ekf.getStatus();
    }

    /** Field-center pose in meters. Null until the first accepted camera correction. */
    public Pose2d getEstimatedPosition() {
        return ekf.isSeeded() ? ekf.getEstimatedPosition() : null;
    }

    public String getStatus() { return status; }
}
