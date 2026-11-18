package org.firstinspires.ftc.teamcode.vision;

/** Overturn CELL floor projections in a field-center frame, in millimeters. */
public enum CellTarget {
    BLUE_UPPER(true, true, 17.7 * 25.4, 12.75 * 25.4),
    BLUE_LOWER(true, false, -17.7 * 25.4, 12.75 * 25.4),
    RED_UPPER(false, true, 17.7 * 25.4, -12.75 * 25.4),
    RED_LOWER(false, false, -17.7 * 25.4, -12.75 * 25.4);

    public final boolean blueAlliance;
    public final boolean upperHalf;
    public final double xMm;
    public final double yMm;

    CellTarget(boolean blueAlliance, boolean upperHalf, double xMm, double yMm) {
        this.blueAlliance = blueAlliance;
        this.upperHalf = upperHalf;
        this.xMm = xMm;
        this.yMm = yMm;
    }

    public static CellTarget forRobotX(boolean blueAlliance, double robotXmm) {
        if (!Double.isFinite(robotXmm)) return null;
        boolean upper = robotXmm >= 0.0;
        if (blueAlliance) return upper ? BLUE_UPPER : BLUE_LOWER;
        return upper ? RED_UPPER : RED_LOWER;
    }
}
