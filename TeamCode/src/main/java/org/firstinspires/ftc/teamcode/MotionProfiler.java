package org.firstinspires.ftc.teamcode;

public class MotionProfiler {
    private double x0, x1, vMax, aMax;
    private double tAccel, tCruise, tDecel, totalTime;
    private double d, dAccel;

    public MotionProfiler(double x0, double x1, double vMax, double aMax) {
        this.x0 = x0;
        this.x1 = x1;
        this.vMax = vMax;
        this.aMax = aMax;
        this.d = Math.abs(x1 - x0);

        dAccel = (vMax * vMax) / (2 * aMax);
        if (2 * dAccel < d) {
            // Trapezoidal
            tAccel = vMax / aMax;
            tCruise = (d - 2 * dAccel) / vMax;
            tDecel = tAccel;
        } else {
            // Triangular
            tAccel = Math.sqrt(d / aMax);
            vMax = aMax * tAccel;
            tCruise = 0;
            tDecel = tAccel;
        }
        totalTime = tAccel + tCruise + tDecel;
    }

    public MotionState getState(double t) {
        double direction = Math.signum(x1 - x0);

        if (t < tAccel) {
            // Acceleration
            double a = aMax * direction;
            double v = a * t;
            double x = x0 + 0.5 * a * t * t;
            return new MotionState(x, v, a);
        } else if (t < tAccel + tCruise) {
            // Cruise
            double v = vMax * direction;
            double x = x0 + direction * (dAccel + v * (t - tAccel));
            return new MotionState(x, v, 0);
        } else if (t < totalTime) {
            // Deceleration
            double tDec = t - tAccel - tCruise;
            double a = -aMax * direction;
            double v = vMax * direction + a * tDec;
            double x = x1 - 0.5 * a * tDec * tDec;
            return new MotionState(x, v, a);
        } else {
            return new MotionState(x1, 0, 0);
        }
    }

    public double getTotalTime() {
        return totalTime;
    }

    class MotionState {
        public final double x, v, a;

        public MotionState(double x, double v, double a) {
            this.x = x;
            this.v = v;
            this.a = a;
        }
    }
}
