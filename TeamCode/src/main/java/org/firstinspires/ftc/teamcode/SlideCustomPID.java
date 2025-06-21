package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="SlideCustomPID")
public class SlideCustomPID extends LinearOpMode {
    private TelemetryPacket packet;
    private FtcDashboard dashboard;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    public static int target;

    private double Kp = 0;
    private double Kd = 0;

    private int lastError = 0;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(target, leftSlide.getCurrentPosition());

            leftSlide.setPower(power);
            rightSlide.setPower(power);

            packet.put("left pos ", leftSlide.getCurrentPosition());
            packet.put("right pos ", rightSlide.getCurrentPosition());
            packet.put("target ", target);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double PIDControl(int target, int current) {
        int error = target - current;

        double derivative = (error-lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd);
        return output;
    }
}