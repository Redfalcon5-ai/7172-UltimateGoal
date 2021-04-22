package org.firstinspires.ftc.teamcode.drive.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;
import org.firstinspires.ftc.teamcode.util.DualPad;

@TeleOp

public class CrowleyTeleOp extends LinearOpMode {
    RobotHardwareOB robot = new RobotHardwareOB();
    DualPad gpad = new DualPad();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean aLast = false;
        boolean bLast = false;
        boolean wgflip = false;
        boolean wgopen = false;

        waitForStart();

        double target = 1.7;
        while (opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            double jy = -gpad.left_stick_y - gpad.right_stick_y; // forward
            double jx = gpad.right_stick_x;  // strafing
            double jw = gpad.left_stick_x;   // turning
            if (gpad.dpad_down) {
                robot.driveYXP(jy, jx, target);
            } else {
                robot.driveYXW(jy, jx, jw);
            }
            //if (gpad.y) target = robot.getIMUHeading();
            if (gpad.y) target = robot.getPixyV();

            /*
            if (gpad.dpad_up) robot.setFireVelocity(robot.SHOOTER_VELOCITY_LOW);
            if (gpad.dpad_left) robot.setFireVelocity(robot.SHOOTER_VELOCITY_LOW);
            if (gpad.dpad_right) robot.setFireVelocity(robot.SHOOTER_VELOCITY_LOW);

             */
            
            boolean aThis = gpad.a;
            if (aThis && !aLast) {
                wgflip = !wgflip;
                if (wgflip) robot.wgFlip();
                else robot.wgStow();
            }
            aLast = aThis;
            
            boolean bThis = gpad.b;
            if (bThis && !bLast) {
                wgopen = !wgopen;
                if (wgopen) robot.wgOpen();
                else robot.wgClose();
            }
            bLast = bThis;

            if (gpad.left_bumper) robot.fire();
            if (gpad.right_trigger > 0.25) robot.intake();
            if (gpad.right_bumper) robot.outtake(); 
            if (gpad.x) robot.quiet();
            if (gpad.left_bumper) robot.fire();

            robot.updateAll();
            telemetry.addData("lrangeV", robot.getLRangeV());
            telemetry.addData("pixyV", robot.getPixyV());
            telemetry.addData("isRingLoaded", robot.isRingLoaded());
            telemetry.addData("heading0", robot.getIMUHeading());
            telemetry.addData("heading1", robot.getIMU1Heading());
            telemetry.addData("rangeV", robot.getLRangeV());
            double vel = robot.shooter1.getVelocity();
            telemetry.addData("velerror", vel - robot.fireVelocity);
            
            telemetry.update();
        }
    }
    
}
