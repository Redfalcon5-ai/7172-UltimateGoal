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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Arrays;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Disabled
@Autonomous(group="Mar20RedAuto")
public class Mar20RedAuto_TFOD extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardware robot   = new RobotHardware();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AcvtjYf/////AAABmehCu3IEgElchwvMigjFqD1s9omMDs5F01lEo3FVqEIg/l5XQlHGj9MgXekDJiOt9m4WamftoEqEUUHlx9pbqW01bmole7jyWAU50dipOfpJ75c4k04Bnscb6RJkbcacd9JpgNTNngaCJiYJ3E6ZyJ3ay4pvwnBmcLPopk+UbI/igXNbCX0TVWED91OwFgy/aRIW2o3srpk9ACTqOG7CH8AzABCbQljzv5ML+B6lwCK8vGTAO1pABAdDC/KCArkjbWKMvbI3lDnNHA4mWuNi1zsO6XZZss5t+3FtnqY2iW078V5YQHOEOnldSTQjfW65/L6NYgm4yHT8GZSRiS7U4eoWBgumFDM9TFXOYZ74MFJU";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    String ringPos;
    int objDetected;
    double right;
    double left;
    double bottom;
    double top;

    @Override
    public void runOpMode() {
        //Init hardware map and set motor directions + servo postions
        robot.init(hardwareMap);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.grabber.setPosition(0);
        robot.tilt.setPosition(0.63);

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
        Thread VufThread = new vuThread();



        //Set RR start Pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new com.acmerobotics.roadrunner.geometry.Pose2d(-61.5, -32.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


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
                .strafeTo(new Vector2d(-10, -26))
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(24, -50, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(0, -40, Math.toRadians(180)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-24, -40, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one4 = drive.trajectoryBuilder(one3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-24, -55, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-32, -59, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(27, -33, Math.toRadians(160)),
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
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(-35, -54, Math.toRadians(0)))
                .build();

        Trajectory four3 = drive.trajectoryBuilder(four2.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(-47, -35, Math.toRadians(0)))
                .build();

        Trajectory four4 = drive.trajectoryBuilder(four3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-36, -35, Math.toRadians(10)),
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
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(39, -47, Math.toRadians(0)))
                .build();

        robot.indexer.setPosition(0.5);


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        VufThread.start();

        //Determine the number of rings in the starter stack
        int rings = 0;

        if(ringPos.equals("Four")){
            rings = 4;
        }
        else if(ringPos.equals("One")){
            rings = 1;
        }
        else if(ringPos.equals("None")){
            rings = 0;
        }


        robot.shooter1.setVelocity(-1600);
        robot.conveyor.setPower(-0.75);
        robot.indexer.setPosition(0.5);


        drive.followTrajectory(move1);
        drive.followTrajectory(move2);




        //Shot 1
        robot.indexer.setPosition(0.85);
        sleep(250);
        robot.indexer.setPosition(0.5);

        sleep(1500);

        //Shot 2
        robot.indexer.setPosition(0.85);
        sleep(250);
        robot.indexer.setPosition(0.5);

        robot.conveyor.setPower(-1);
        sleep(1000);

        //Shot 3
        robot.indexer.setPosition(0.85);
        sleep(500);
        robot.indexer.setPosition(0.5);
        sleep(1000);

        robot.shooter1.setVelocity(0);
        robot.conveyor.setPower(0);


        if (rings == 0){
            drive.followTrajectory(zero1);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            robot.wobble.setVelocity(750);
            sleep(1000);
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
            robot.tilt.setPosition(0.61);
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
            sleep(1000);
            robot.grabber.setPosition(0.025);
            sleep(1000);
            robot.intake.setPower(0);
            robot.shooter1.setVelocity(-1600);
            drive.followTrajectory(one6);

            robot.indexer.setPosition(1);
            sleep(300);
            robot.indexer.setPosition(0.5);


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
            sleep(1000);
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

            sleep(2000);

            robot.indexer.setPosition(1);
            sleep(500);
            robot.indexer.setPosition(0.5);

            drive.followTrajectory(four5);

            sleep(1000);

            robot.indexer.setPosition(1);
            sleep(500);
            robot.indexer.setPosition(0.5);


            sleep(500);

            robot.indexer.setPosition(1);
            sleep(500);
            robot.indexer.setPosition(0.5);

            sleep(500);

            robot.indexer.setPosition(1);
            sleep(500);
            robot.indexer.setPosition(0.5);

            drive.followTrajectory(four6);
            robot.grabber.setPosition(0.5);
            drive.followTrajectory(four7);
            robot.wobble.setVelocity(-750);
            sleep(500);
            robot.wobble.setVelocity(0);
            //telemetry.addData("h", recognition.getlabel());
        }


        VufThread.interrupt();
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

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
                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            objDetected = updatedRecognitions.size();
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                ringPos = recognition.getLabel();
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                top = recognition.getTop();
                                left = recognition.getLeft();
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                right = recognition.getBottom();
                                bottom = recognition.getRight();
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }

}

