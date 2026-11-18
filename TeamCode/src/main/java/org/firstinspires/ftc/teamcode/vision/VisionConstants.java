package org.firstinspires.ftc.teamcode.vision;

/** Limelight acquisition settings. Geometry must match the actual uploaded fmaps. */
public final class VisionConstants {
    private VisionConstants() { }
    public static final int bluePipeline = 0;
    public static final int redPipeline = 1;
    public static final int pollRateHz = 50;
    public static final long maxFrameAgeMs = 200;
    // BIOBUZZ manual V0.9 section 9.6.1: pivot axis 43.95 inches above the mat.
    public static final double hivePivotHeightMeters = 43.95 * 0.0254;
    public static final double hivePivotXmeters = 0.0;
    public static final double maxHiveTiltDegrees = 35.0;
    public static final double maxRobotLevelErrorDegrees = 10.0;
    public static final double maxFloorErrorMeters = 0.15;
    public static final int minimumTagCount = 2;
    public static final double minimumVisionXyStdMeters = 0.08;
    public static final double visionDistanceNoiseCoefficient = 0.04;
    public static final double visionHeadingStdDegrees = 20.0;
}
