package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp (name="TestMotionProfiler")
public class TestMotionProfiler extends OpMode {
    MotionProfiler profiler;
    ElapsedTime timer;

    DcMotor left, right;

    public static double kP, kV, kA;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "leftSlide");
        right = hardwareMap.get(DcMotor.class, "rightSlide");

        profiler  = new MotionProfiler(
                0,
                2500,
                1000,
                500
        );
        timer = new ElapsedTime();

        kP = 0.8;
        kV = 
    }

    @Override
    public void loop() {
        double t = timer.seconds();
        MotionProfiler.MotionState profilerTarget = profiler.getState(t);

        double error = profilerTarget.x - left.getCurrentPosition();
        double output = kP * error + kV * profilerTarget.v + kA * profilerTarget.a;
    }
}
