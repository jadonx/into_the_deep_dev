package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slides {
    // Actual hardware
    private DcMotor left;
    private DcMotor right;

    // The PID controller to control the slides
    private PID pid;

    // PID variables, proportional and derivative respectively
    private double kP;
    private double kD;
    private double kG;

    // OpMode to access hardwareMap so we can assign the hardware to the variables
    private OpMode opMode;

    // Constructor class, this is used to assign all the values when creating a new Slides class
    public Slides(double kp, double kd, double kg, OpMode opmode) {
        // Assigning variables
        kP = kp;
        kD = kd;

        opMode = opmode;

        pid = new PID(kP, kD, kG);

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
        // Reset from RTP
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = pid.PIDControl(target, currentPos());

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

    public int currentPos() {
        // We choose to use the left motor as the lead motor
        return left.getCurrentPosition();
    }

    public void updateValues(double kp, double kd, double kg) {
        pid.updateValues(kp, kd, kg);
    }
}
