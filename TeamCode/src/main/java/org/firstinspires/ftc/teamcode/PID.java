package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {
    private double kP;

    private double kV, kA;

    private int lastError;

    private ElapsedTime timer;

    public PID(double kp) {
        kP = kP;

        timer = new ElapsedTime();

        lastError = 0;
    }

    public double PIDControl(int target, int current) {
        int error = target - current;

        lastError = error;

        // feedforward
        // double ff = (target > current) ? kG : -kG;

        return (error * kP);
    }

    public double PIDControlMP(int target, int current, MotionProfiler profile, ElapsedTime timer) {
        double t = timer.seconds();
        MotionProfiler.MotionState profileTarget = profile.getState(t);

        double error = profileTarget.x - current;
        double output = kP * error + kV * profileTarget.v + kA * profileTarget.a;

        return output;
    }

    public void updateValues(double kp) {
        kP = kp;
    }
}
