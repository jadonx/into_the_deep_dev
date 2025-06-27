package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    // Actual hardware
    private DcMotor left;
    private DcMotor right;

    // The PID controller to control the slides
    private PID pid;

    // Motion Profiler
    private MotionProfiler motionProfiler;

    // PID variables, proportional and derivative respectively
    private double kP;

    // OpMode to access hardwareMap so we can assign the hardware to the variables
    private OpMode opMode;

    // Constructor class, this is used to assign all the values when creating a new Slides class
    public Slides(double kp, OpMode opmode) {
        // Assigning variables
        kP = kp;

        opMode = opmode;

        pid = new PID(kP);

        // Assigning hardware
        left = opMode.hardwareMap.get(DcMotor.class, "leftSlide");
        right = opMode.hardwareMap.get(DcMotor.class, "rightSlide");

        // Motor settings

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Resets encoders after every running of the program
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double moveSlidesPID(int target) {
        resetRTP();

        double power = pid.PIDControl(target, currentPos());

        left.setPower(power);
        right.setPower(power);

        return power;
    }

    public double moveSlidesMP(int target, int vMax, int aMax) {
        resetRTP();

        // Calculate initial trapezoidal/triangular motion plan
        MotionProfiler profiler = new MotionProfiler(currentPos(), target, vMax, aMax);

        double power = pid.PIDControlMP(target, currentPos(), profiler, new ElapsedTime());

        left.setPower(power);
        right.setPower(power);

        return power;
    }

    public void moveSlidesRTP(int target) {
        left.setTargetPosition(target);
        right.setTargetPosition(target);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(1);
        right.setPower(1);
    }

    public void resetRTP() {
        // Reset from RTP
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int currentPos() {
        // We choose to use the left motor as the lead motor
        return left.getCurrentPosition();
    }

    public void updateValues(double kp) {
        pid.updateValues(kp);
    }
}
