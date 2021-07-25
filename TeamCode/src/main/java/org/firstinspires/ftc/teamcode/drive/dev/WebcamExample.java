package org.firstinspires.ftc.teamcode.drive.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.UGBasicHighGoalPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
public class WebcamExample extends LinearOpMode {
    OpenCvCamera webcam;

    RobotHardware robot = new RobotHardware();
    double turretPos = 0.3;

    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;
    double REDx1 = 241;
    double REDy1 = 0.352;
    double REDx2 = 62;
    double REDy2 = 0.522;
    double BLUEx1 = 157;
    double BLUEy1 = 0.481;
    double BLUEx2 = 248;
    double BLUEy2 = 0.394;
    boolean turretSet = false;
    boolean red = true;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter1.setVelocityPIDFCoefficients(100, 0.75, 0, 0);
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
                flyPow = -1560;
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
                flyPow = -1560;
                conveyorPow = -2;
            }

            if (gpad.y) {
                turretSet = !turretSet;
            }
            if (gpad.b) {
                red = !red;
            }

            if (turretSet) {
                if (gpad.dpad_right && turretPos > 0.275) {
                    turretPos -= 0.001;
                }
                if (gpad.dpad_left && turretPos < 0.63) {
                    turretPos += 0.001;
                }
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

            if (pipeline.isRedVisible() && red) {
                Rect redRect = pipeline.getRedRect();
                Point centerOfRedGoal = pipeline.getCenterofRect(redRect);

                double targetPos = getTargetPos(REDx1, REDy1, REDx2, REDy2, centerOfRedGoal.x);

                telemetry.addData("Red goal position",
                        centerOfRedGoal.toString());
                telemetry.addData("Target Pos", targetPos);

                if (!turretSet)
                    turretPos = targetPos;

            }


            if (pipeline.isBlueVisible() && !red) {
                Rect blueRect = pipeline.getBlueRect();
                Point centerOfBlueGoal = pipeline.getCenterofRect(blueRect);

                double targetPos = getTargetPos(BLUEx1, BLUEy1, BLUEx2, BLUEy2, centerOfBlueGoal.x);

                telemetry.addData("Blue goal position",
                        centerOfBlueGoal.toString());
                telemetry.addData("Target Pos", targetPos);

                if (!turretSet)
                    turretPos = targetPos;
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
            telemetry.addData("Set", turretSet);
            telemetry.addData("Red", red);
            telemetry.update();

        }

    }

    //Method for Mecanum Drive
    public void mecDrive(double forward, double forward2, double turn, double strafe) {
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method to get motor powers
    public double motorPow(double x){
        if(x>1){
            return x-1;
        }
        if(x<-1){
            return x+1;
        }
        return x;
    }

    //Method to get servo powers
    public double servoPow(double x){
        return (motorPow(x)*0.4) + 0.5;
    }

    public double getTargetPos(double x1, double y1, double x2, double y2, double xCoor){
        double slope = (y1 - y2)/(x1 - x2);
        double yIntercept = y1 - slope*x1;
        double returnPos = slope*xCoor + yIntercept;

        return returnPos;
    }
}