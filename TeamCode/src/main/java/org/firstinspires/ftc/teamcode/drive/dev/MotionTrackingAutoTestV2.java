package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;


@Autonomous(name="MotionTrackingAutoTestV2", group="TXCRedAuto")
public class MotionTrackingAutoTestV2 extends LinearOpMode
{
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Set RR start Pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(MotionTrackingTestV2.moves.get(0).getPos());


        waitForStart();

        for(int i = 1; i < MotionTrackingTest.moves.size(); i++){
            Trajectory x = drive.trajectoryBuilder(MotionTrackingTestV2.moves.get(i-1).getPos())
                    .lineToLinearHeading(
                            new com.acmerobotics.roadrunner.geometry.Pose2d(MotionTrackingTestV2.moves.get(i).getPos().getX(), MotionTrackingTestV2.moves.get(i).getPos().getY(), Math.toRadians(MotionTrackingTestV2.moves.get(i).getPos().getHeading())),
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



