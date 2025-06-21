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

    // FTC Dash
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // Testing Values
    public static int target;

    @Override
    public void init() {
        slides = new Slides(kP, kD, this);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // PID
        slides.updateValues(kP, kD);
        slides.moveSlides(target);

        packet.put("target ", target);
        packet.put("current ", slides.currentPos());

        dashboard.sendTelemetryPacket(packet);
    }
}
