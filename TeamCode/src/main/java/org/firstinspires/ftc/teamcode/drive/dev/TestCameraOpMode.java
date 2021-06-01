package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        telemetry.addData("X", up.pose.getTranslation().getX());
        telemetry.addData("Y", up.pose.getTranslation().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}
