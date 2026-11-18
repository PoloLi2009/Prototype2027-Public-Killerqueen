package org.firstinspires.ftc.teamcode.vision.fusion;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import java.util.Map;
import java.util.TreeMap;

/** Interpolates Pinpoint pose in meters and radians at a monotonic timestamp. */
public class PoseHistory {
    public static final double DURATION_SECONDS = 1.5;
    private final TreeMap<Double, Pose2d> samples = new TreeMap<>();
    public void clear() { samples.clear(); }
    public void add(double time, Pose2d odometry) {
        samples.put(time, odometry);
        while (samples.size() > 2 && samples.firstKey() < time - DURATION_SECONDS)
            samples.pollFirstEntry();
    }
    public Pose2d sample(double time) {
        Map.Entry<Double, Pose2d> a = samples.floorEntry(time), b = samples.ceilingEntry(time);
        if (a == null || b == null) return null; // Never silently clamp an old/future measurement.
        if (a.getKey().equals(b.getKey())) return a.getValue();
        double t = (time - a.getKey()) / (b.getKey() - a.getKey());
        Twist2d twist = a.getValue().log(b.getValue());
        return a.getValue().exp(new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t));
    }
}
