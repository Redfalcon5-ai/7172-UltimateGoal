package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DualPad;


@TeleOp

public class TestOp3 extends LinearOpMode {
    RobotHardwareAS robot = new RobotHardwareAS();
    DualPad gpad = new DualPad();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    //Create T265 Camera Object
    private static T265Camera slamra = null;

    //Set Camera's Position on the robot (Mount with wire on the left)
    String CameraPos = "left";
    String left = "left";
    String right = "right";
    String back = "back";
    String front = "front";

    //Instance Variables for threading
    double Y = 0;
    double X = 0;
    double Heading = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);

        //Create T265 Thread
        Thread t265Thread = new T265Thread();

        //Init SLAM and the camera with the correct starting coordinates
        initCameraPos();

        boolean aLast = false;
        boolean bLast = false;
        boolean wgflip = false;
        boolean wgopen = false;

        sleep(5000);

        try{
            slamra.start();
        } catch (Exception e){
            try{
                slamra.start();
            } catch (Exception f){
                slamra.start();
            }
        }

        waitForStart();

        //Start thread and continue with the main thread
        t265Thread.start();

        double targetHeading = 9999;
        double zeroHeading = 0;
        while (opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            double jy = -gpad.left_stick_y - gpad.right_stick_y; // forward
            double jx = gpad.right_stick_x;  // strafing
            double jw = gpad.left_stick_x;   // turning
            if (jx >= 0.5 || jx <= -0.5) targetHeading = zeroHeading;
            if (jw != 0) targetHeading = 9999;
            if (wgflip || targetHeading == 9999)
                robot.driveYXW(jy, jx, jw);
            else
                //robot.driveYXH(jy, jx, targetHeading);
                robot.driveYXH(jy, jx, targetHeading, Heading);
            if (gpad.y) zeroHeading = Heading;

            if (gpad.right_trigger > 0.25) robot.intake();
            if (gpad.left_trigger > 0.25) robot.outtake();
            if (gpad.x) robot.quiet();
            if (gpad.right_bumper) robot.fire();

            boolean aThis = gpad.a;
            if (aThis && !aLast) {
                wgflip = !wgflip;
                if (wgflip) robot.wgFlip();
                else robot.wgStow();
            }
            aLast = aThis;

            boolean bThis = gpad.b;
            if (bThis && !bLast) {
                wgopen = !wgopen;
                if (wgopen) robot.wgOpen();
                else robot.wgClose();
            }
            bLast = bThis;

            if (gpad.dpad_up) robot.setFireVelocity(robot.SHOOTER_VELOCITY_NORMAL);
            if (gpad.dpad_down) robot.setFireVelocity(robot.SHOOTER_VELOCITY_LOW);



            robot.updateAll();

            double x = 1200;
            double y = 2000;

            if(robot.isShooterReady()){
                dashboardTelemetry.addData("Shooter Ready", 1400);
            }
            else{
                dashboardTelemetry.addData("Shooter Ready", 1300);
            }

            dashboardTelemetry.addData("velocity", robot.shooter1.getVelocity());
            dashboardTelemetry.addData("x", x);
            dashboardTelemetry.addData("y", y);
            dashboardTelemetry.update();
        }

        t265Thread.interrupt();
        slamra.stop();
    }

    public void mecDrive(double forward, double forward2, double turn, double strafe) {
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method to Init SLAM and the camera with the correct starting coordinates
    public void initCameraPos(){
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        if(CameraPos.equals(left)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-31 * 0.0254, -12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(right)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(31 * 0.0254, 12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(back)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-12 * 0.0254, 31 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(front)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(12 * 0.0254, -31 * 0.0254, Rotation2d.fromDegrees(0)));
        }
    }

    //Thread Class
    private class T265Thread extends Thread {
        public T265Thread()
        {
            this.setName("T265Thread");
        }

        @Override
        public void run()
        {

            while (!isInterrupted())
            {
                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

                if (up == null) return;



                if(CameraPos.equals(left)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (-1)*(up.pose.getTranslation().getY() / 0.0254);
                    X = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                    Heading = (up.pose.getHeading()) * (57.295);
                }

                if(CameraPos.equals(right)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (up.pose.getTranslation().getY() / 0.0254);
                    X = (up.pose.getTranslation().getX() / 0.0254);
                }

                if(CameraPos.equals(back)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                    X = (up.pose.getTranslation().getY() / 0.0254);
                }

                if(CameraPos.equals(front)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (up.pose.getTranslation().getX() / 0.0254);
                    X = (-1)*(up.pose.getTranslation().getY() / 0.0254);
                }


                Translation2d translation = new Translation2d(X, Y);
                Rotation2d rotation = up.pose.getRotation();
            }
        }
    }
}

