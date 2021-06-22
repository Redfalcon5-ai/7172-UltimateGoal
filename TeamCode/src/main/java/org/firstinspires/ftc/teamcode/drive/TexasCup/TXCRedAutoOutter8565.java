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
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline2;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.util.RobotHardwareOB.ShootMode.LOAD;


@Autonomous(name="TXCRedAutoOutter8565", group="TXCRedAuto")
public class TXCRedAutoOutter8565 extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardwareOB robot   = new RobotHardwareOB();
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

            if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.FOUR && !ringsSet){
                rings = 4;
            }
            else if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.ONE && !ringsSet){
                rings = 1;
            }
            else if(pipeline.position == RingDeterminationPipeline2.SkystoneDeterminationPipeline.RingPosition.NONE && !ringsSet){
                rings = 0;
            }

            telemetry.addData("Rings", pipeline.position);
            telemetry.addData("Ring Path", rings);
            telemetry.addData("Auto Path", choiceO);
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sleep(15000);


        //Universal trajectories
        Trajectory move1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-7, -56.5, Math.toRadians(0
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

        Trajectory move3 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-10, -16.5, Math.toRadians(0
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

        Trajectory move4 = drive.trajectoryBuilder(move3.end())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(-10, -22, Math.toRadians(0
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(2, -65, Math.toRadians(0)),
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
                        new com.acmerobotics.roadrunner.geometry.Pose2d(22, -25, Math.toRadians(0)),
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



        robot.telemetry = telemetry;
        telemetry.log().setCapacity(16);
        robot.setTarget(0,0);
        robot.setShootMode(LOAD);
        robot.updateAll();

        drive.followTrajectory(move1);
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
                startPose = zero1.end();
            }

            if (rings == 1) {
                drive.followTrajectory(one1);
                sleep(250);
                robot.wgOpen();
                robot.updateAll();
                startPose = one1.end();
            }

            if (rings == 4) {
                drive.followTrajectory(four1);
                sleep(250);
                robot.wgOpen();
                robot.updateAll();
                startPose = four1.end();
            }

        }

        Trajectory move9_2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(
                        new com.acmerobotics.roadrunner.geometry.Pose2d(6, -36    , Math.toRadians(0)),
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

        robot.dropDown();

        robot.wgStow();
        robot.updateAll();

        sleep(2000);





    }



}



