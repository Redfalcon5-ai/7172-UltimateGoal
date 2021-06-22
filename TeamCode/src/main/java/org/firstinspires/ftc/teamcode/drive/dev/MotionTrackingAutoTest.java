package org.firstinspires.ftc.teamcode.drive.dev;

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


@Autonomous(name="MotionTrackingAutoTest", group="TXCRedAuto")
public class MotionTrackingAutoTest extends LinearOpMode
{
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Set RR start Pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(MotionTrackingTest.moves.get(0));


        waitForStart();

        for(int i = 1; i < MotionTrackingTest.moves.size(); i++){
            Trajectory x = drive.trajectoryBuilder(MotionTrackingTest.moves.get(i-1))
                    .lineToLinearHeading(
                            new com.acmerobotics.roadrunner.geometry.Pose2d(MotionTrackingTest.moves.get(i).getX(), MotionTrackingTest.moves.get(i).getY(), Math.toRadians(MotionTrackingTest.moves.get(i).getHeading())),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            drive.followTrajectory(x);
        }






    }



}



