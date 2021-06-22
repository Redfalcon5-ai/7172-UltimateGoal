package org.firstinspires.ftc.teamcode.drive.Crowley;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardwareAS;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Disabled
@Autonomous(name="CrowleyBlueAuto", group="CrowleyRedAuto")
public class CrowleyBlueAuto extends LinearOpMode
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
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new com.acmerobotics.roadrunner.geometry.Pose2d(9, 41, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(27, 47, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();



        //Zero ring trajectories
        Trajectory zero1 = drive.trajectoryBuilder(move1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(62, 70, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(30, 45.5, Math.toRadians(90)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(70, 85, Math.toRadians(200)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory zero4 = drive.trajectoryBuilder(zero3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(77, 50, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //One ring trajectories
        Trajectory one1 = drive.trajectoryBuilder(move1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(37, 40, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one2 = drive.trajectoryBuilder(one1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(37, 46, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(98, 40, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one4 = drive.trajectoryBuilder(one3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(37, 33, Math.toRadians(40)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(92, 50, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one6 = drive.trajectoryBuilder(one5.end())
                .forward(10)
                .build();


        //Four ring trajectories
        Trajectory four1 = drive.trajectoryBuilder(move1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(36, 38, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four2 = drive.trajectoryBuilder(four1.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(36, 45, Math.toRadians(0)))
                .build();

        Trajectory four3 = drive.trajectoryBuilder(four2.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(42, 36, Math.toRadians(0)))
                .build();

        Trajectory four4 = drive.trajectoryBuilder(four3.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(38, 45, Math.toRadians(0)))
                .build();

        Trajectory four5 = drive.trajectoryBuilder(four4.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(120, 65, Math.toRadians(5)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory four6 = drive.trajectoryBuilder(four5.end())
                .back(20)
                .build();

        Trajectory four7 = drive.trajectoryBuilder(four6.end())
                .lineToLinearHeading(new com.acmerobotics.roadrunner.geometry.Pose2d(110, 65, Math.toRadians(180)))
                .build();

        Trajectory four8 = drive.trajectoryBuilder(four7.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(16, -47, Math.toRadians(5)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
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

        robot.shooter(1600);
        robot.setArms(false);

        drive.followTrajectory(move1);

        robot.outtake();
        robot.updateAll(1600);
        robot.updateAll(1600);

        sleep(500);

        robot.updateAll(1600);
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll(1600);
        }

        robot.intake();
        robot.updateAll(1600);
        robot.updateAll(1600);
        sleep(500);

        //Shot 2
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll(1600);
        }

        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll(1600);
        }

        robot.intake();
        robot.updateAll(1600);
        robot.updateAll(1600);
        sleep(500);

        //Shot 3
        robot.fire();
        while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
            robot.updateAll(1600);
        }

        sleep(500);

        robot.quiet();
        robot.updateAll(1600);


        if (rings == 0){
            robot.outtake();
            robot.updateAll(1600);
            drive.followTrajectory(zero1);
            robot.wgOpen();
            robot.updateAll();
            sleep(1000);
            robot.wgFlip();
            robot.updateAll();
            drive.followTrajectory(zero2);
            sleep(1000);
            robot.wgClose();
            robot.updateAll();
            sleep(1000);
            drive.followTrajectory(zero3);
            robot.wgOpen();
            robot.updateAll();
            sleep(1000);
            robot.wgStow();
            robot.updateAll();
            sleep(1000);
            drive.followTrajectory(zero4);
        }




        if (rings == 1){
            robot.shooter(1580);
            robot.intake();
            robot.updateAll(1580);
            drive.followTrajectory(one1);
            drive.followTrajectory(one2);

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1580);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1580);
            }

            sleep(1000);

            drive.followTrajectory(one3);
            robot.quiet();
            robot.wgOpen();
            robot.updateAll();
            robot.wgFlip();
            robot.updateAll();
            robot.intake();
            robot.updateAll();
            sleep(1000);

            drive.followTrajectory(one4);
            robot.wgClose();
            robot.updateAll();
            sleep(1000);

            drive.followTrajectory(one5);
            robot.wgOpen();
            robot.updateAll();
            robot.wgStow();
            robot.updateAll();
            drive.followTrajectory(one6);

        }



        if (rings == 4){
            drive.followTrajectory(four1);
            for(int i = 0; i < 3; i++){
                robot.intake();
                robot.updateAll(1600);
                sleep(450);
                robot.updateAll(1600);
                sleep(450);
            }

            drive.followTrajectory(four2);

            robot.intake();
            robot.updateAll(1600);
            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.intake();
            robot.updateAll(1600);
            sleep(500);

            //Shot 2
            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            drive.followTrajectory(four3);
            robot.intake();
            robot.updateAll(1600);
            sleep(1500);
            drive.followTrajectory(four4);

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.quiet();
            robot.updateAll();

            drive.followTrajectory(four5);
            robot.wgOpen();
            robot.updateAll();
            robot.wgFlip();
            robot.updateAll();
            drive.followTrajectory(four6);
            /*
            robot.wgClose();
            robot.updateAll();
            drive.followTrajectory(four7);
            robot.wgOpen();
            robot.wgStow();
            robot.updateAll();



            robot.wgOpen();
            robot.updateAll(1600);
            robot.wgFlip();
            robot.updateAll(1600);
            drive.followTrajectory(four2);
            drive.followTrajectory(four3);
            sleep(500);
            robot.wgClose();
            robot.updateAll(1600);
            sleep(500);
            drive.followTrajectory(four4);
            robot.shooter(1600);



            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }
            robot.intake();
            robot.updateAll(1600);

            drive.followTrajectory(four6);

            sleep(500);

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            robot.fire();
            while(robot.smode != RobotHardwareOB.ShootMode.LOAD){
                robot.updateAll(1600);
            }

            sleep(500);

            drive.followTrajectory(four7);
            robot.wgOpen();
            robot.updateAll(1600);
            robot.wgStow();
            robot.updateAll(1600);

            drive.followTrajectory(four8);
            */


        }






    }


}

