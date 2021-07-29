package org.firstinspires.ftc.teamcode.drive.MTI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOBV2;
import org.opencv.core.Point;
import org.opencv.core.Rect;

@TeleOp

public class MTIOpBlue extends LinearOpMode {
    RobotHardwareOBV2 robot = new RobotHardwareOBV2();
    DualPad gpad = new DualPad();

    int lastShot = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.setTargetColor(2);
        robot.startTargetingCamera();

        boolean aLast = false;
        boolean bLast = false;
        boolean xLast = false;
        
        boolean wgflip = false;
        boolean wgopen = false;
        
        boolean dpadLast = false;
        boolean backLast = false;
        
        boolean armDown = false;
        
        waitForStart();
        telemetry.log().setCapacity(16);
        // robot.telemetry = telemetry
        robot.grabber.setPosition(0.025);

        double targetH = robot.getHeading();
        while (opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            double jy = -gpad.left_stick_y - gpad.right_stick_y; // forward
            double jx = gpad.right_stick_x;  // strafing
            double jw = gpad.left_stick_x;   // turning
            
            boolean backThis = gpad.back && !gpad.x;
            backLast = backThis; 

            //Drive Modes
            if (gpad.dpad_up && !gpad.x) {
                robot.driveYXH(jy, jx, targetH);
            } else {
                robot.driveYXW(jy, jx, jw);
            }

            //Set Heading
            if (gpad.y) {
                targetH = robot.getIMUHeading();
            }

            //Color
            if(gpad.back && !gpad.x){ robot.setTargetColor(1); }
            if(gpad.backShift){ robot.setTargetColor(2); }
            if(gpad.dpad_downShift){ robot.setTargetColor(3);}

            //Adjust Ring Path
            if (gpad.x && !dpadLast) {
                if (gpad.dpad_left) robot.adjustTurret(-0.008);
                if (gpad.dpad_right) robot.adjustTurret(+0.008);
                if (gpad.dpad_up) robot.adjustShooter(+20);
                if (gpad.dpad_down) robot.adjustShooter(-20);
            }

            dpadLast = gpad.dpad_left || gpad.dpad_right
                    || gpad.dpad_up || gpad.dpad_down;

            //Set Targets
            if(gpad.dpad_down && !gpad.x) robot.setTarget(0);
            if(gpad.yShift) robot.setTarget(1);
            if(gpad.dpad_leftShift) robot.setTarget(2);
            if(gpad.dpad_upShift) robot.setTarget(3);
            if(gpad.dpad_rightShift) robot.setTarget(4);

            //Reset Turret
            if(gpad.back && gpad.x){
                robot.adjustTurret(0);
                robot.adjustShooter(0);
            }

            //Wobble Goal Controls
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

            //X button
            boolean xThis = gpad.x;
            xLast = xThis;

            //Shooting and Intake Controls
            if (gpad.left_bumper) robot.fire();
            if (gpad.right_trigger > 0.25) robot.intake();
            if (gpad.right_bumper) robot.outtake(); 
            if (gpad.xShift) robot.quiet();

            //OpenCV Telemtry
            if (robot.pipeline.isRedVisible()) {
                Rect redRect = robot.pipeline.getRedRect();
                Point centerOfRedGoal = robot.pipeline.getCenterofRect(redRect);

                telemetry.addData("Red goal position",
                        centerOfRedGoal.toString());
            }
            if (robot.pipeline.isBlueVisible()) {
                Rect blueRect = robot.pipeline.getBlueRect();
                Point centerOfBlueGoal = robot.pipeline.getCenterofRect(blueRect);

                telemetry.addData("Blue goal position",
                        centerOfBlueGoal.toString());
            }

            //Telemtry
            robot.updateAll();
            telemetry.addData("turretPos", robot.turretPosition);
            telemetry.addData("turretAdjust", robot.turretAdjust);
            telemetry.addData("robotTarget", robot.shootTarget);
            telemetry.addData("fireVelocity", robot.fireVelocity);
            telemetry.addData("shooterAdjust", robot.shooterAdjust);
            double vel = robot.shooter1.getVelocity();
            telemetry.addData("velerror", vel - robot.fireVelocity);
            telemetry.addData("isRingLoaded", robot.isRingLoaded());
            telemetry.addData("Heading", robot.getHeading());
            telemetry.addData("targetServoPos", robot.getTargetServoPos());
            telemetry.addData("Color", robot.getTargetColor());
            telemetry.addData("Center", robot.goalPos);
            telemetry.addData("Width", robot.goalWidth);
            telemetry.update();
        }
    }
    
}
