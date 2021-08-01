package org.firstinspires.ftc.teamcode.drive.MTI;

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
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline2;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOBV2;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOBV2.ShootMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;


@Autonomous(name="MTIBlueAutoInner", group="MTIRedAuto")
public class MTIBlueAutoInner extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardwareOBV2 robot   = new RobotHardwareOBV2();
    ElapsedTime runtime = new ElapsedTime();

    //OpenCV stuff
    RingDeterminationPipeline2 rings   = new RingDeterminationPipeline2();
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
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
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new com.acmerobotics.roadrunner.geometry.Pose2d(-61.5, 33, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-33, 42, Math.toRadians(0)))
                .build();

        Trajectory move2 = drive.trajectoryBuilder(move1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, 42, Math.toRadians(0
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-4, 65, Math.toRadians(210)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(24, 48, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory one2 = drive.trajectoryBuilder(one1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(0, 58, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(50, 64, Math.toRadians(240)),
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
        robot.setAutonCamera(4);

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

            if (gamepad1.left_bumper){
                robot.setAutonCamera(-0.0001);
            }

            if (gamepad1.right_bumper){
                robot.setAutonCamera(0.0001);
            }

            telemetry.addData("Rings", pipeline.position);
            telemetry.addData("Ring Path", rings);
            telemetry.addData("Camera Pos", robot.cameraPos);
            telemetry.update();
        }

        webcam.closeCameraDevice();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.dropDown();
        robot.startTargetingCamera();
        robot.updateAll();

        robot.telemetry = telemetry;
        telemetry.log().setCapacity(16);
        robot.setTarget(0);
        robot.setTargetColor(2);
        robot.setShootMode(ShootMode.LOAD);
        robot.updateAll();

        runtime.reset();
        while(runtime.seconds() < 0.25 && opModeIsActive()){
            robot.updateAll();
        }

        drive.followTrajectory(move1);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.intake();
        robot.updateAll();
        sleep(2000);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.intake.setPower(1);

        drive.followTrajectory(move2);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        robot.autoFire();
        sleep(150);

        if(rings == 0){
            drive.followTrajectory(zero1);
            robot.wgStow();
            robot.updateAll();

            sleep(1000);

            robot.wgOpen();
            robot.updateAll();

            sleep(500);
        }
        if(rings == 1){
            drive.followTrajectory(one1);
            robot.wgStow();
            robot.updateAll();

            sleep(1000);

            robot.wgOpen();
            robot.updateAll();

            sleep(500);
        }
        if(rings == 4){
            drive.followTrajectory(four1);
            robot.wgStow();
            robot.updateAll();

            sleep(1000);

            robot.wgOpen();
            robot.updateAll();

            sleep(500);
        }

        sleep(500);

        if(rings == 4) {
            Trajectory move3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(
                            new com.acmerobotics.roadrunner.geometry.Pose2d(-35, 45.5, Math.toRadians(0
                            )),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            drive.followTrajectory(move3);
        }

        if(rings == 1) {
            Trajectory move3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(
                            new com.acmerobotics.roadrunner.geometry.Pose2d(-40, 54.5, Math.toRadians(10
                            )),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            drive.followTrajectory(move3);
        }

        if(rings == 0) {
            Trajectory move3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(
                            new com.acmerobotics.roadrunner.geometry.Pose2d(-35, 49.5, Math.toRadians(10
                            )),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            drive.followTrajectory(move3);
        }

        robot.wgClose();
        robot.updateAll();

        sleep(1000);

        //Zero ring trajectories
        Trajectory zero2 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(2, 60, Math.toRadians(230)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //One ring trajectories
        Trajectory one3 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(35, 40, Math.toRadians(200)),
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
        Trajectory four2 = drive.trajectoryBuilder(move2.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(40, 83, Math.toRadians(200)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        if(rings == 0){
            drive.followTrajectory(zero2);
            robot.wgOpen();
            robot.updateAll();
        }
        if(rings == 1){
            drive.followTrajectory(one3);
            robot.wgOpen();
            robot.updateAll();
        }
        if(rings == 4){
            drive.followTrajectory(four2);
            robot.wgOpen();
            robot.updateAll();
        }

        robot.wgFlip();
        robot.updateAll();

        sleep(1000);

        Trajectory move4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(16, 12, Math.toRadians(0
                        )),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        drive.followTrajectory(move4);

        robot.dropUp();
    }

    public void wait(int x){
        sleep(x);
    }



}



