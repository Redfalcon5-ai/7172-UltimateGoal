package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Disabled
@Autonomous(group="Mar20RedAuto_VuforiaThread")
public class Mar20RedAuto_VuforiaThread extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardware robot   = new RobotHardware();

    //OpenCV stuff
    RingDeterminationPipeline rings   = new RingDeterminationPipeline();
    RingDeterminationPipeline.SkystoneDeterminationPipeline pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
    OpenCvCamera webcam;

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

    double Y = 0;
    double X = 0;
    double Heading = 0;

    @Override
    public void runOpMode() {
        //Init hardware map and set motor directions + servo postions
        robot.init(hardwareMap);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.grabber.setPosition(0);
        robot.tilt.setPosition(0.65);




        //Start OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {

                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }
                                     }
        );

        //Set RR start Pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new com.acmerobotics.roadrunner.geometry.Pose2d(-61.5, -32.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

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


        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose)
                .splineTo(
                        new Vector2d(-10, -16), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory move2 = drive.trajectoryBuilder(move1.end())
                .strafeTo(new Vector2d(-10, -23))
                .build();

        //Zero ring trajectories
        Trajectory zero1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(5, -60, Math.toRadians(-50)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory zero2 = drive.trajectoryBuilder(zero1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-32, -58.5, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory zero3 = drive.trajectoryBuilder(zero2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(5, -50, Math.toRadians(-210)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory zero4 = drive.trajectoryBuilder(zero3.end())
                .forward(1)
                .build();

        Trajectory zero5 = drive.trajectoryBuilder(zero4.end())
                .strafeRight(10)
                .build();

        //One ring trajectories
        Trajectory one1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(28, -50, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one2 = drive.trajectoryBuilder(one1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(0, -45, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one3 = drive.trajectoryBuilder(one2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-24, -42, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one4 = drive.trajectoryBuilder(one3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-24, -58, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one5 = drive.trajectoryBuilder(one4.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-31, -58, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one6 = drive.trajectoryBuilder(one5.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-10, -30, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one7 = drive.trajectoryBuilder(one6.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(25, -33, Math.toRadians(160)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one8 = drive.trajectoryBuilder(one7.end())
                .forward(10)
                .build();

        //Four ring trajectories
        Trajectory four1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(56, -46, Math.toRadians(-90)))
                .build();

        Trajectory four2 = drive.trajectoryBuilder(four1.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(-35, -57, Math.toRadians(0)))
                .build();

        Trajectory four3 = drive.trajectoryBuilder(four2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-47, -35, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four4 = drive.trajectoryBuilder(four3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-36, -35, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four5 = drive.trajectoryBuilder(four4.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-12, -35, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four6 = drive.trajectoryBuilder(four5.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(48, -47, Math.toRadians(160)))
                .build();

        Trajectory four7 = drive.trajectoryBuilder(four6.end())
                .forward(10)
                .build();


        telemetry.addData("Rings", pipeline.position);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VufThread.start();
        targetsUltimateGoal.activate();

        //Determine the number of rings in the starter stack
        int rings = 0;

        if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.FOUR){
            rings = 4;
        }
        else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.ONE){
            rings = 1;
        }
        else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.NONE){
            rings = 0;
        }

        robot.shooter1.setVelocity(-2000);
        robot.indexer.setPosition(0.5);

        drive.followTrajectory(move1);
        drive.followTrajectory(move2);

        robot.conveyor.setPower(-0.75);

        //Shot 1
        robot.indexer.setPosition(0.25);
        sleep(100);
        boolean ringShot = false;
        while(!ringShot){
            if(robot.touch.getState()){
                robot.indexer.setPosition(0.25);
            }
            else if (!robot.touch.getState()){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }
        }

        sleep(500);

        //Shot 2
        robot.indexer.setPosition(0.25);
        sleep(100);
        ringShot = false;
        while(!ringShot){
            if(robot.touch.getState()){
                robot.indexer.setPosition(0.25);
            }
            else if (!robot.touch.getState()){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }
        }

        sleep(500);

        //Shot 3
        robot.indexer.setPosition(0.25);
        sleep(100);
        ringShot = false;
        while(!ringShot){
            if(robot.touch.getState()){
                robot.indexer.setPosition(0.25);
            }
            else if (!robot.touch.getState()){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }
        }

        robot.shooter1.setVelocity(0);
        robot.conveyor.setPower(0);

        if (rings == 0){
            drive.followTrajectory(zero1);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            robot.wobble.setVelocity(750);
            sleep(7000);
            robot.wobble.setVelocity(0);
            drive.followTrajectory(zero2);
            robot.grabber.setPosition(0.025);
            sleep(1000);
            drive.followTrajectory(zero3);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            drive.followTrajectory(zero4);
            robot.wobble.setVelocity(-750);
            sleep(1000);
            robot.wobble.setVelocity(0);
            drive.followTrajectory(zero5);
        }

        if (rings == 1){
            drive.followTrajectory(one1);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            robot.wobble.setVelocity(750);
            sleep(1000);
            robot.wobble.setVelocity(0);
            robot.intake.setPower(-1);
            robot.conveyor.setPower(-1);
            drive.followTrajectory(one2);
            drive.followTrajectory(one3);
            drive.followTrajectory(one4);
            drive.followTrajectory(one5);
            robot.grabber.setPosition(0.025);
            sleep(1000);
            robot.intake.setPower(0);
            robot.shooter1.setVelocity(-2000);
            drive.followTrajectory(one6);

            robot.indexer.setPosition(0.25);
            sleep(200);
            ringShot = false;
            while(!ringShot){
                if(robot.touch.getState()){
                    robot.indexer.setPosition(0.25);
                }
                else if (!robot.touch.getState()){
                    robot.indexer.setPosition(0.5);
                    ringShot = true;
                }
            }

            drive.followTrajectory(one7);
            robot.shooter1.setVelocity(0);
            robot.conveyor.setPower(0);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            drive.followTrajectory(one8);
            robot.wobble.setVelocity(-750);
            sleep(1000);
            robot.wobble.setVelocity(0);
        }

        if (rings == 4){
            drive.followTrajectory(four1);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            robot.wobble.setVelocity(750);
            sleep(500);
            robot.wobble.setVelocity(0);
            drive.followTrajectory(four2);
            sleep(500);
            robot.grabber.setPosition(0.025);
            sleep(500);
            drive.followTrajectory(four3);
            robot.intake.setPower(-0.75);
            robot.conveyor.setPower(-0.75);
            robot.shooter1.setVelocity(-2000);
            drive.followTrajectory(four4);
            robot.intake.setPower(0);

            sleep(1000);

            robot.indexer.setPosition(0.25);
            sleep(200);
            ringShot = false;
            while(!ringShot){
                if(robot.touch.getState()){
                    robot.indexer.setPosition(0.25);
                }
                else if (!robot.touch.getState()){
                    robot.indexer.setPosition(0.5);
                    robot.intake.setPower(-0.5);
                    ringShot = true;
                }
            }

            drive.followTrajectory(four5);

            sleep(1000);

            robot.indexer.setPosition(0.25);
            sleep(200);
            ringShot = false;
            while(!ringShot){
                if(robot.touch.getState()){
                    robot.indexer.setPosition(0.25);
                }
                else if (!robot.touch.getState()){
                    robot.indexer.setPosition(0.5);
                    ringShot = true;
                }
            }

            sleep(500);

            robot.indexer.setPosition(0.25);
            sleep(200);
            ringShot = false;
            while(!ringShot){
                if(robot.touch.getState()){
                    robot.indexer.setPosition(0.25);
                }
                else if (!robot.touch.getState()){
                    robot.indexer.setPosition(0.5);
                    ringShot = true;
                }
            }

            sleep(500);

            robot.indexer.setPosition(0.25);
            sleep(200);
            ringShot = false;
            while(!ringShot){
                if(robot.touch.getState()){
                    robot.indexer.setPosition(0.25);
                }
                else if (!robot.touch.getState()){
                    robot.indexer.setPosition(0.5);
                    ringShot = true;
                }
            }

            drive.followTrajectory(four6);
            robot.grabber.setPosition(0.5);
            drive.followTrajectory(four7);
            robot.wobble.setVelocity(-750);
            sleep(500);
            robot.wobble.setVelocity(0);
        }

        VufThread.interrupt();
        targetsUltimateGoal.deactivate();


    }

    //Thread for updating pos
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
                            telemetry.addData("Visible Target", trackable.getName());
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
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                        Y = translation.get(1) / mmPerInch + 8;
                        X = translation.get(0) / mmPerInch - 7;
                        telemetry.addData("X", X);
                        telemetry.addData("Y", Y);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        Heading = rotation.thirdAngle - 90;
                        telemetry.addData("heading", Heading);
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                    }
                    telemetry.update();
                }
            }
        }
    }


}

