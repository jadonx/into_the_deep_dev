package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {
    private double kP;
    private double kD;

    private int lastError;

    private ElapsedTime timer;

    public PID(double kp, double kd) {
        kP = kP;
        kD = kD;

        lastError = 0;
        timer = new ElapsedTime();
    }

    public double PIDControl(int target, int current) {
        int error = target - current;

        double derivative = (error-lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * kP) + (derivative * kD);
    }

    public void updateValues(double kp, double kd) {
        kP = kp;
        kD = kd;
    }
}
