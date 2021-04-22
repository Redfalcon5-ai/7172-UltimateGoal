package org.firstinspires.ftc.teamcode.drive.Hedrick;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Templates.T265DriveSample;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Mar20RedTeleOp", group="Linear Opmode")
public class Mar20RedTeleOp extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();


    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double linearPos = 0.66;
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;
    double grabberPow = 0.0;

    //Create T265 Camera Object
    private static T265Camera slamra = null;

    //Set Camera's Position on the robot (Mount with wire on the left)
    String CameraPos = "left";
    String left = "left";
    String right = "right";
    String back = "back";
    String front = "front";

    //Vuforia Stuff
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AcvtjYf/////AAABmehCu3IEgElchwvMigjFqD1s9omMDs5F01lEo3FVqEIg/l5XQlHGj9MgXekDJiOt9m4WamftoEqEUUHlx9pbqW01bmole7jyWAU50dipOfpJ75c4k04Bnscb6RJkbcacd9JpgNTNngaCJiYJ3E6ZyJ3ay4pvwnBmcLPopk+UbI/igXNbCX0TVWED91OwFgy/aRIW2o3srpk9ACTqOG7CH8AzABCbQljzv5ML+B6lwCK8vGTAO1pABAdDC/KCArkjbWKMvbI3lDnNHA4mWuNi1zsO6XZZss5t+3FtnqY2iW078V5YQHOEOnldSTQjfW65/L6NYgm4yHT8GZSRiS7U4eoWBgumFDM9TFXOYZ74MFJU";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    List<VuforiaTrackable> allTrackables;

    //Instance Variables for threading
    double Y = 0;
    double X = 0;
    double Heading = 0;

    @Override
    public void runOpMode() {
        //Initialize Hardware and reverse motors
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
        robot.shooter1.setVelocityPIDFCoefficients(15,0.75,0,0);
        robot.indexer.setDirection(Servo.Direction.REVERSE);
        robot.tilt.setPosition(0.66);

        //Create T265 Thread
        Thread t265Thread = new T265Thread();

        //Init SLAM and the camera with the correct starting coordinates
        initCameraPos();

        //More Vuforia Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        Thread VufThread = new vuThread();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(X, Y, Math.toRadians(Heading)));

        waitForStart();

        VufThread.start();
        targetsUltimateGoal.activate();

        slamra.start();

        //Start thread and continue with the main thread
        t265Thread.start();

        while(opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            //Driving Controls
            double leftStickY = -gpad.left_stick_y; //Forward and Backward
            double rightStickY = -gpad.right_stick_y;   //Forward and Backward
            double leftStickX = gpad.left_stick_x;  //Turning
            double rightStickX = gpad.right_stick_x;    //Strafing
            mecDrive(leftStickY, rightStickY, leftStickX, rightStickX);

            drive.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(-X, -Y, Math.toRadians(Heading)));
            com.acmerobotics.roadrunner.geometry.Pose2d myPos = drive.getPoseEstimate();

            //Latch Controls
            if (conveyorPow >= -1 && conveyorPow <= 1) {
                conveyorPow = 0;
            }
            //if (flyPow >= -1 && flyPow <= 1) {
            //    flyPow = 0;
            //}
            if (intakePow >= -1 && intakePow <= 1) {
                intakePow = 0;
            }
            if (true) {
                grabberPow = 0;
            }


            //Distance Sensor controls
            if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
                flyPow = -1600;
                conveyorPow = -1.75;
            }



            //Button Controls
            if (gpad.a) {
                robot.grabber.setPosition(0.025);
            }
            if (gpad.aShift) {
                grabberPow = 750;
            }
            if (gpad.b) {
                robot.grabber.setPosition(0.5);
            }
            if (gpad.bShift) {
                grabberPow = -750;
            }
            if (gpad.y){
                Trajectory shoot = drive.trajectoryBuilder(myPos)
                        .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(0, -30, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(shoot);
            }

            //Bumper and Touch Switch Controls
            double indexerPos = 0.85;
            if (gpad.right_bumper) indexerPos = 0.5;

            //D-pad Controls
            if (gpad.dpad_up && linearPos < 0.9) {
                linearPos = linearPos + 0.001;
            }
            if (gpad.dpad_down && linearPos > 0.1) {
                linearPos = linearPos - 0.001;
            }
            if (gpad.dpad_left) {
                linearPos = 0.742;
            }
            if (gpad.dpad_right) {
                linearPos = 0.672;
            }

            //Unlatch
            if (gpad.x) {
                flyPow = 0;
                conveyorPow = 0;
            }
            if (gpad.xShift) {
                flyPow = -1600;
                conveyorPow = -1.75;
            }

            //Trigger Controls
            if (gamepad1.right_trigger > 0) {
                intakePow = -1;
                if(conveyorPow != -1.75){
                    conveyorPow = -0.75;
                }
            }
            if (gamepad1.left_trigger > 0) {
                intakePow = 1;
                if(conveyorPow != -1.75){
                    conveyorPow = 0.75;
                }
            }

            //Set Powers
            robot.conveyor.setPower(motorPow(conveyorPow));
            robot.shooter1.setVelocity(flyPow);
            robot.intake.setPower(motorPow(intakePow));
            robot.wobble.setVelocity(grabberPow);
            robot.tilt.setPosition(linearPos);
            robot.indexer.setPosition(indexerPos);

            //Display the coordinates of the robot (inches)
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.addData("Heading", Heading);
            telemetry.addData("RRX", myPos.getX());
            telemetry.addData("RRY", myPos.getY());
            telemetry.addData("RRHeading", myPos.getHeading());
            telemetry.update();
        }

        t265Thread.interrupt();
        slamra.stop();
    }

    //Method for X-Drive
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

    //Method to Init SLAM and the camera with the correct starting coordinates
    public void initCameraPos(){
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
    }

    //Thread (T265) for updating pos
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

                if(!targetVisible) {
                    if (CameraPos.equals(left)) {
                        // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                        X = (-1) * (up.pose.getTranslation().getY() / 0.0254);
                        Y = (-1) * (up.pose.getTranslation().getX() / 0.0254);
                        Heading = (up.pose.getHeading()) * (-57.295);
                    }

                    if (CameraPos.equals(right)) {
                        // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                        X = (up.pose.getTranslation().getY() / 0.0254);
                        Y = (up.pose.getTranslation().getX() / 0.0254);
                    }

                    if (CameraPos.equals(back)) {
                        // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                        X = (-1) * (up.pose.getTranslation().getX() / 0.0254);
                        Y = (up.pose.getTranslation().getY() / 0.0254);
                    }

                    if (CameraPos.equals(front)) {
                        // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                        X = (up.pose.getTranslation().getX() / 0.0254);
                        Y = (-1) * (up.pose.getTranslation().getY() / 0.0254);
                    }
                }

                else if(targetVisible){
                    slamra.setPose(new Pose2d(Y * 0.0254,X * 0.0254, Rotation2d.fromDegrees(Heading)));
                }


                Translation2d translation = new Translation2d(X, Y);
                Rotation2d rotation = up.pose.getRotation();
            }
        }
    }

    //Thread (Vuforia) for updating pos
    private class vuThread extends Thread {
        public vuThread()
        {
            this.setName("vuThread");
        }

        @Override
        public void run()
        {

            while (!isInterrupted())
            {
                while (!isStopRequested()) {

                    // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        Y = translation.get(1) / mmPerInch;
                        X = translation.get(0) / mmPerInch - 8;

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        Heading = rotation.thirdAngle - 90;

                        slamra.setPose(new Pose2d(Y * 0.0254, X * 0.0254, Rotation2d.fromDegrees(Heading)));
                    }
                }
            }
        }
    }

}




