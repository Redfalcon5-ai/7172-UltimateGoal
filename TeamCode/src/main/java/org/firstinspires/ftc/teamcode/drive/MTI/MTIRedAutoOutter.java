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
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline2;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOBV2;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOBV2.ShootMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;


@Autonomous(name="MTIRedAutoOutter", group="MTIRedAuto")
public class MTIRedAutoOutter extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardwareOBV2 robot   = new RobotHardwareOBV2();
    ElapsedTime runtime = new ElapsedTime();

    //OpenCV stuff
    RingDeterminationPipeline2 rings   = new RingDeterminationPipeline2();
    RingDeterminationPipeline2.SkystoneDeterminationPipeline pipeline = new RingDeterminationPipeline2.SkystoneDeterminationPipeline();
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
        pipeline = new RingDeterminationPipeline2.SkystoneDeterminationPipeline();
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
        com.acmerobotics.roadrunner.geometry.Pose2d startPose = new com.acmerobotics.roadrunner.geometry.Pose2d(-61.5, -56.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, -56.5, Math.toRadians(0
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

        Trajectory move2 = drive.trajectoryBuilder(move1.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, -36, Math.toRadians(0
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(2, -66, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(22, -41, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(0, -58, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(55, -43, Math.toRadians(-60)),
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
        robot.setAutonCamera(1);

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

            if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.FOUR && !ringsSet){
                rings = 4;
            }
            else if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.ONE && !ringsSet){
                rings = 1;
            }
            else if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.NONE && !ringsSet){
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

        robot.startTargetingCamera();
        robot.updateAll();

        robot.telemetry = telemetry;
        telemetry.log().setCapacity(16);
        robot.setTarget(0);
        robot.setTargetColor(1);
        robot.setShootMode(ShootMode.LOAD);
        robot.updateAll();

        drive.followTrajectory(move1);
        drive.followTrajectory(move2);

        runtime.reset();
        while(runtime.seconds() < 0.25 && opModeIsActive()){
            robot.updateAll();
        }

        robot.fire();
        while(robot.smode != ShootMode.LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != ShootMode.LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != ShootMode.LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while(robot.smode != ShootMode.LOAD){
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        if(rings == 0){
            drive.followTrajectory(zero1);
            robot.wgOpen();
            robot.updateAll();
        }
        if(rings == 1){
            drive.followTrajectory(one1);
            robot.wgOpen();
            robot.updateAll();
            drive.followTrajectory(one2);
        }
        if(rings == 4){
            drive.followTrajectory(four1);
            robot.wgOpen();
            robot.updateAll();
        }

        Trajectory move3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-43, -55, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory move4 = drive.trajectoryBuilder(move3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-43, -32, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory move5 = drive.trajectoryBuilder(move4.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-26, -32, Math.toRadians(0)))
                .build();

        Trajectory move6 = drive.trajectoryBuilder(move5.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-10, -35, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        drive.followTrajectory(move3);
        robot.dropDown();
        drive.followTrajectory(move4);
        drive.followTrajectory(move5);

        robot.intake();
        robot.updateAll();

        sleep(1000);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.intake();
        robot.updateAll();

        drive.followTrajectory(move6);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(250);

        robot.fire();
        while (robot.smode != RobotHardwareOBV2.ShootMode.LOAD) {
            robot.updateAll();
        }
        telemetry.update();
        sleep(500);

        robot.quiet();
        robot.updateAll();




        Trajectory move7 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(8, -32   , Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        drive.followTrajectory(move7);

        robot.dropDown();
        robot.updateAll();
        sleep(750);

        robot.dropUp();
        robot.updateAll();
        sleep(750);
    }



}



