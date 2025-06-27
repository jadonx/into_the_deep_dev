package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {
    private double kP;
    private double kD;
    private double kG;

    private int lastError;

    private ElapsedTime timer;

    public PID(double kp, double kd, double kg) {
        kP = kP;
        kD = kD;
        kG = kg;

        lastError = 0;
        timer = new ElapsedTime();
    }

    public double PIDControl(int target, int current) {
        int error = target - current;

        double derivative = (error-lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        // feedforward
        double ff = (target > current) ? kG : -kG;

        return (error * kP) + (derivative * kD) + ff;
    }

    public void updateValues(double kp, double kd, double kg) {
        kP = kp;
        kD = kd;
        kG = kg;
    }
}
