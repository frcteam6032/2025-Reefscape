package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class ObstacleAvoidance {
    public static class Readings {
        // Use infinity if theres no obstacle
        public double front = Double.POSITIVE_INFINITY;
        public double frontLeft = Double.POSITIVE_INFINITY;
        public double frontRight = Double.POSITIVE_INFINITY;
        public double left = Double.POSITIVE_INFINITY;
        public double right = Double.POSITIVE_INFINITY;
    }

    // Basically return a vector that nudges the bot away from obstacles
    public static Translation2d repel(Readings r) {
        final double kMin = 0.25;      // How close u can be to an object
        final double kHard = 0.10;     // Imminent collision
        final double kMaxNudge = 1.0;  // Max

        double[] d = {0, 0};
        java.util.function.BiConsumer<Double, Translation2d> add =
            (dist, normal) -> {
                if (dist.isInfinite() || dist <= 0) return;
                if (dist < kMin) {
                    double mag = (1.0/Math.max(dist, kHard) - 1.0/kMin);
                    d[0] += normal.getX() * mag; 
                    d[1] += normal.getY() * mag; 
                }
            };

        add.accept(r.front,      new Translation2d(-1,  0)); // push backward
        add.accept(r.frontLeft,  new Translation2d(-0.9, 0.3));
        add.accept(r.frontRight, new Translation2d(-0.9,-0.3));
        add.accept(r.left,       new Translation2d( 0,   1));
        add.accept(r.right,      new Translation2d( 0,  -1));

        double mag = Math.hypot(d[0], d[1]);
        if (mag > 1e-6) {
            double scale = Math.min(kMaxNudge, mag) / mag;
            d[0] *= scale; d[1] *= scale;
        }
        return new Translation2d(d[0], d[1]);
    }
}
