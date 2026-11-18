package org.firstinspires.ftc.teamcode.shoot;

/**
 * 瞄点位置取 KQ 既有值（X=60 in，Y 按车位置在 60/84 in 之间切换，72±1 in 迟滞）；
 * 偏航误差与距离严格按 OT VisionMath.coordinateYawErrorDegrees 计算，
 * 发射口相对车头的偏角为 180°（ShootConstants.SHOOTER_YAW_DEGREES）。
 */
public final class ShotTarget {
    /** OT coordinateYawErrorDegrees 的退化判定：距离 ≤1e-6 in 时方向未定义。 */
    private static final double MIN_DISTANCE_INCHES = 1e-6;

    private static final double TARGET_X_IN = 60.0;
    private static final double LOWER_TARGET_Y_IN = 60.0;
    private static final double UPPER_TARGET_Y_IN = 84.0;
    private static final double SWITCH_Y_IN = 72.0;
    private static final double SWITCH_HYSTERESIS_IN = 1.0;

    private double targetYInches = LOWER_TARGET_Y_IN;
    private double distanceInches = Double.NaN;
    private double desiredHeadingDegrees = Double.NaN;
    private double yawErrorDegrees = Double.NaN;

    public void update(double robotXInches, double robotYInches, double robotHeadingRadians) {
        if (!Double.isFinite(robotXInches) || !Double.isFinite(robotYInches)
                || !Double.isFinite(robotHeadingRadians)) {
            invalidate();
            return;
        }
        // KQ 既有的上下半场瞄点切换。
        if (targetYInches == LOWER_TARGET_Y_IN) {
            if (robotYInches > SWITCH_Y_IN + SWITCH_HYSTERESIS_IN) {
                targetYInches = UPPER_TARGET_Y_IN;
            }
        } else if (robotYInches < SWITCH_Y_IN - SWITCH_HYSTERESIS_IN) {
            targetYInches = LOWER_TARGET_Y_IN;
        }
        update(robotXInches, robotYInches, robotHeadingRadians, TARGET_X_IN, targetYInches);
    }

    /** Robot and target positions must use the same field coordinate frame. */
    public void update(double robotXInches, double robotYInches, double robotHeadingRadians,
                       double targetXInches, double targetYInches) {
        invalidate();
        double dx = targetXInches - robotXInches;
        double dy = targetYInches - robotYInches;
        if (!Double.isFinite(dx) || !Double.isFinite(dy) || !Double.isFinite(robotHeadingRadians)) return;
        if (Math.hypot(dx, dy) <= MIN_DISTANCE_INCHES) {
            return;
        }
        distanceInches = Math.hypot(dx, dy);
        desiredHeadingDegrees = Math.IEEEremainder(
                Math.toDegrees(Math.atan2(dy, dx)) - ShootConstants.SHOOTER_YAW_DEGREES, 360.0);
        double error = Math.atan2(dy, dx) - Math.toRadians(ShootConstants.SHOOTER_YAW_DEGREES)
                - robotHeadingRadians;
        yawErrorDegrees = Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
    }

    public void invalidate() {
        distanceInches = Double.NaN;
        desiredHeadingDegrees = Double.NaN;
        yawErrorDegrees = Double.NaN;
    }

    public boolean isValid() {
        return Double.isFinite(yawErrorDegrees) && Double.isFinite(distanceInches);
    }

    public double getTargetXInches() { return TARGET_X_IN; }
    public double getTargetYInches() { return targetYInches; }
    public double getDistanceInches() { return distanceInches; }
    public double getDesiredHeadingDegrees() { return desiredHeadingDegrees; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
}
