package org.firstinspires.ftc.teamcode.drive.TexasCup;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.util.RobotHardwareOB.ShootMode.LOAD;


@Autonomous(name="TXCBlueAutoOutterNoPIXY", group="TXCRedAuto")
public class TXCBlueAutoOutterNoPIXY extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardwareOB robot   = new RobotHardwareOB();
    ElapsedTime runtime = new ElapsedTime();

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
        com.acmerobotics.roadrunner.geometry.Pose2d startPose1 = new com.acmerobotics.roadrunner.geometry.Pose2d(-61.5, 56.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose1);

        int rings = 0;
        boolean ringsSet = false;

        int choiceO = 0;

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

            if(gamepad1.dpad_left){
                choiceO = 0;
            }
            if(gamepad1.dpad_up){
                choiceO = 1;
            }
            if(gamepad1.dpad_right){
                choiceO = 2;
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
            telemetry.addData("Ring Path", rings);
            telemetry.addData("Auto Path", choiceO);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose1)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, 56.5, Math.toRadians(0
                        )),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        robot.telemetry = telemetry;
        telemetry.log().setCapacity(16);
        robot.setTarget(1,5);
        robot.setShootMode(LOAD);
        robot.updateAll();

        drive.followTrajectory(move1);

        Trajectory move2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, 46, Math.toRadians(0
                        )),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //Zero ring trajectories
        Trajectory zero1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(2, 65, Math.toRadians(0)),
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
        Trajectory one1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(22, 33, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //Four ring trajectories
        Trajectory four1 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(50, 64, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        drive.followTrajectory(move2);

        runtime.reset();
        while(runtime.seconds() < 0.25 && opModeIsActive()){
            robot.updateAll();
        }

        robot.fire();
        while(robot.smode != LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        if(choiceO == 0 || choiceO == 1) {

            if (rings == 0) {
                drive.followTrajectory(zero1);
                sleep(250);
                robot.wgOpen();
                robot.updateAll();
                startPose1 = zero1.end();
            }

            if (rings == 1) {
                drive.followTrajectory(one1);
                sleep(250);
                robot.wgOpen();
                robot.updateAll();
                startPose1 = one1.end();
            }

            if (rings == 4) {
                drive.followTrajectory(four1);
                sleep(250);
                robot.wgOpen();
                robot.updateAll();
                startPose1 = four1.end();
            }

            if(rings != 0) {

                robot.setFireVelocity(1600);

                Trajectory move1_01 = drive.trajectoryBuilder(startPose1)
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-37, 56, Math.toRadians(0)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectory(move1_01);

                Trajectory move2_01 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-37, 31, Math.toRadians(0)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                Trajectory move3_01 = drive.trajectoryBuilder(move2_01.end())
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-22, 31, Math.toRadians(5)))
                        .build();

                drive.followTrajectory(move2_01);
                robot.dropDown();
                drive.followTrajectory(move3_01);

                robot.intake();
                robot.updateAll();

                sleep(1000);

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);


                robot.intake();
                robot.updateAll();

                Trajectory move4_01 = drive.trajectoryBuilder(move3_01.end())
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-10, 31, Math.toRadians(5)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectory(move4_01);

                robot.wgFlip();
                robot.updateAll();

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);

                robot.fire();
                while (robot.smode != LOAD) {
                    robot.updateAll();
                }
                telemetry.update();
                sleep(250);

                robot.quiet();
                robot.updateAll();

            }

            if(choiceO == 0) {

                Trajectory move5_0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-34, 27, Math.toRadians(45)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                Trajectory move12_0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(
                                new com.acmerobotics.roadrunner.geometry.Pose2d(-33, 18, Math.toRadians(10)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                if(rings == 0){
                    robot.wgFlip();
                    robot.updateAll();
                    drive.followTrajectory(move12_0);
                }
                else {
                    drive.followTrajectory(move5_0);
                }

                robot.wgClose();
                robot.updateAll();

                sleep(500);

                if (rings == 0) {
                    Trajectory move6_0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(
                                    new com.acmerobotics.roadrunner.geometry.Pose2d(10, 47, Math.toRadians(-90)),
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                            .build();

                    drive.followTrajectory(move6_0);

                    robot.wgOpen();
                    robot.updateAll();
                    robot.wgStow();
                    robot.updateAll();
                }

                if (rings == 1) {
                    Trajectory move7_0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(
                                    new com.acmerobotics.roadrunner.geometry.Pose2d(20, 40, Math.toRadians(180)),
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                            .build();
                    drive.followTrajectory(move7_0);

                    robot.wgOpen();
                    robot.updateAll();
                    robot.wgStow();
                    robot.updateAll();
                }

                if (rings == 4) {
                    Trajectory move8_0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(
                                    new com.acmerobotics.roadrunner.geometry.Pose2d(58, 72, Math.toRadians(-180)),
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                            .build();
                    drive.followTrajectory(move8_0);

                    robot.wgOpen();
                    robot.updateAll();
                    robot.wgStow();
                    robot.updateAll();
                }
            }
        }

        Trajectory move9_2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(11, 12   , Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        drive.followTrajectory(move9_2);

        robot.wgStow();
        robot.updateAll();

        sleep(2000);





    }



}



