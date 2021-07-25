package org.firstinspires.ftc.teamcode.drive.Hedrick;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotHardwareAS;
import org.firstinspires.ftc.teamcode.util.UGBasicHighGoalPipeline;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp


public class PIDTune extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    OpenCvCamera webcam;

    RobotHardware robot = new RobotHardware();
    double turretPos = 0.3;

    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;
    double x1 = 72;
    double y1 = 0.565;
    double x2 = 269;
    double y2 = 0.37;

    public static double p = 200;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter1.setVelocityPIDFCoefficients(p, 0.75, 0, 0);
        robot.indexer.setDirection(Servo.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        UGBasicHighGoalPipeline pipeline = new UGBasicHighGoalPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        //Start thread and continue with the main thread


        double targetHeading = 9999;
        double zeroHeading = 0;
        while (opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            robot.turret.setPosition(turretPos);

            while (opModeIsActive()) {
                gpad.mergePads(gamepad1, gamepad2);

                //Driving Controls
                double leftStickY = -gpad.left_stick_y; //Forward and Backward
                double rightStickY = -gpad.right_stick_y;   //Forward and Backward
                double leftStickX = gpad.left_stick_x;  //Turning
                double rightStickX = gpad.right_stick_x;    //Strafing
                mecDrive(leftStickY, rightStickY, leftStickX, rightStickX);

                //Latch Controls
                if (conveyorPow >= -1 && conveyorPow <= 1) {
                    conveyorPow = 0;
                }
                if (intakePow >= -1 && intakePow <= 1) {
                    intakePow = 0;
                }


                //Distance Sensor controls
                if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
                    flyPow = 1500;
                    conveyorPow = -2;
                }

                //Bumper and Touch Switch Controls
                double indexerPos = 0.5;
                if (gpad.right_bumper) indexerPos = 0.0;

                //Unlatch
                if (gpad.x) {
                    flyPow = 0;
                    conveyorPow = 0;
                }
                if (gpad.xShift) {
                    flyPow = 1500;
                    conveyorPow = -2;
                }

                if (gpad.dpad_right && turretPos > 0.275) {
                    turretPos -= 0.005;
                }
                if (gpad.dpad_left && turretPos < 0.63) {
                    turretPos += 0.005;
                }

                //Trigger Controls
                if (gamepad1.right_trigger > 0) {
                    intakePow = -1;
                    if (conveyorPow != -2) {
                        conveyorPow = -1;
                    }
                }
                if (gamepad1.left_trigger > 0) {
                    intakePow = 1;
                    if (conveyorPow != -2) {
                        conveyorPow = 1;
                    }
                }

                if (pipeline.isRedVisible()) {
                    Rect redRect = pipeline.getRedRect();
                    Point centerOfRedGoal = pipeline.getCenterofRect(redRect);

                    telemetry.addData("Red goal position",
                            centerOfRedGoal.toString());
                }

                if (pipeline.isBlueVisible()) {
                    Rect blueRect = pipeline.getBlueRect();
                    Point centerOfBlueGoal = pipeline.getCenterofRect(blueRect);

                    telemetry.addData("Blue goal position",
                            centerOfBlueGoal.toString());
                }

                //Set Powers
                robot.conveyor.setPower(motorPow(conveyorPow));
                robot.shooter1.setVelocity(flyPow);
                robot.intake.setPower(motorPow(intakePow));
                robot.indexer.setPosition(indexerPos);
                robot.turret.setPosition(turretPos);

                /*
                 * Send some stats to the telemetry
                 */


                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("pos", turretPos);
                telemetry.update();

                double x = 1200;
                double y = 2000;

                dashboardTelemetry.addData("velocity", robot.shooter1.getVelocity());
                dashboardTelemetry.addData("x", x);
                dashboardTelemetry.addData("y", y);
                dashboardTelemetry.update();
            }
        }

        //Method for Mecanum Drive


    }

    public void mecDrive (double forward, double forward2, double turn, double strafe){
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method to get motor powers
    public double motorPow ( double x){
        if (x > 1) {
            return x - 1;
        }
        if (x < -1) {
            return x + 1;
        }
        return x;
    }

    //Method to get servo powers
    public double servoPow ( double x){
        return (motorPow(x) * 0.4) + 0.5;
    }
}

