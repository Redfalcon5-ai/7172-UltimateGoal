package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

//

@Autonomous(group="Mar20RedAuto")
public class Mar20RedAuto extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardwareAS robot   = new RobotHardwareAS();

    //OpenCV stuff
    RingDeterminationPipeline rings   = new RingDeterminationPipeline();
    RingDeterminationPipeline.SkystoneDeterminationPipeline pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        //Init hardware map and set motor directions + servo postions
        robot.init(hardwareMap);
        robot.wgClose();
        robot.updateAll();
        robot.tilt.setPosition(0.66);


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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(22, -47, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-32, -61, Math.toRadians(0)),
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
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(60, -46, Math.toRadians(-90)))
                .build();

        Trajectory four2 = drive.trajectoryBuilder(four1.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(-35, -58, Math.toRadians(0)))
                .build();

        Trajectory four3 = drive.trajectoryBuilder(four2.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(-47, -35, Math.toRadians(0)))
                .build();

        Trajectory four4 = drive.trajectoryBuilder(four3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-36, -40, Math.toRadians(5)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-12, -40, Math.toRadians(5)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four6 = drive.trajectoryBuilder(four5.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(48, -47, Math.toRadians(160)))
                .build();

        Trajectory four7 = drive.trajectoryBuilder(four6.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(16, -47, Math.toRadians(0)))
                .build();

        int rings = 0;
        boolean ringsSet = false;
        while(!opModeIsActive() && !isStopRequested()){
            if(gamepad1.x){
                rings = 0;
                ringsSet = true;
            }
            if(gamepad1.y){
                rings = 1;
                ringsSet = true;
            }
            if(gamepad1.b){
                rings = 4;
                ringsSet = true;
            }
            if(gamepad1.a){
                ringsSet = false;
            }

            if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.FOUR && !ringsSet){
                rings = 4;
            }
            else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.ONE && !ringsSet){
                rings = 1;
            }
            else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.NONE && !ringsSet){
                rings = 0;
            }

            telemetry.addData("Rings", pipeline.position);
            telemetry.addData("Auto Path", rings);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.shooter(1700);
        robot.intake();
        robot.updateAll();

        drive.followTrajectory(move1);
        drive.followTrajectory(move2);

        sleep(500);

        robot.updateAll();
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll();
        }

        sleep(500);

        //Shot 2
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll();
        }

        robot.intake();
        robot.updateAll();
        sleep(500);

        //Shot 3
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll();
        }

        sleep(500);

        robot.quiet();
        robot.updateAll();
        robot.intake();
        robot.updateAll();

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
            drive.followTrajectory(one2);
            drive.followTrajectory(one3);
            drive.followTrajectory(one4);
            drive.followTrajectory(one5);
            sleep(1000);
            robot.grabber.setPosition(0.025);
            sleep(1000);
            robot.intake.setPower(0);
            robot.shooter(1600);
            drive.followTrajectory(one6);

            robot.indexer.setPosition(0.5);
            sleep(300);
            robot.indexer.setPosition(0.85);


            drive.followTrajectory(one7);
            robot.shooter(0);
            robot.conveyor.setPower(-1);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            drive.followTrajectory(one8);
            robot.wobble.setVelocity(-750);
            sleep(1000);
            robot.wobble.setVelocity(0);
        }

        if (rings == 4){
            drive.followTrajectory(four1);
            robot.wgOpen();
            robot.updateAll();
            robot.wgFlip();
            robot.updateAll();
            drive.followTrajectory(four2);
            sleep(500);
            robot.wgClose();
            robot.intake();
            robot.updateAll();
            sleep(500);
            drive.followTrajectory(four3);
            drive.followTrajectory(four4);
            robot.shooter(1660);
            robot.updateAll();

            sleep(500);

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll();
            }
            robot.intake();
            robot.updateAll();

            drive.followTrajectory(four5);

            sleep(500);

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll();
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll();
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll();
            }

            sleep(500);

            drive.followTrajectory(four6);
            robot.wgOpen();
            robot.updateAll();
            robot.wgStow();
            robot.updateAll();

            drive.followTrajectory(four7);
        }




    }


}

