package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="SlidesTest")
public class SlidesTest extends OpMode {
    // Declaring slides
    Slides slides;

    // PID values
    public static double kP;
    public static double kD;
    public static double kG;

    // FTC Dash
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // Testing Values
    public static int target;

    // Change mode
    public static boolean isPID;

    @Override
    public void init() {
        slides = new Slides(kP, kD, kG,this);

        target = 0;

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        isPID = true;
    }

    @Override
    public void loop() {
        if (isPID) {
            runPID();
        } else {
            runRTP();
        }

        slides.updateValues(kP, kD, kG);

        packet.put("target ", target);
        packet.put("current ", slides.currentPos());

        dashboard.sendTelemetryPacket(packet);
    }

    public void runPID() {
        packet.put("power ", slides.moveSlidesPID(target));
    }

    public void runRTP() {
        slides.moveSlidesRTP(target);
    }
}
