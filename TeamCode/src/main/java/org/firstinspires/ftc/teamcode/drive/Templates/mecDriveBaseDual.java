package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name="mecDriveBaseDual", group="Linear Opmode")
public class mecDriveBaseDual extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();

    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double linearPos = 0.66;
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;
    double grabberPow = 0.0;

    @Override
    public void runOpMode() {
        //Initialize Hardware and reverse motors
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter1.setVelocityPIDFCoefficients(15,0.75,0,0);
        robot.indexer.setDirection(Servo.Direction.REVERSE);
        robot.tilt.setPosition(0.66);

        waitForStart();

        while(opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            //Driving Controls
            double leftStickY = -gpad.left_stick_y; //Forward and Backward
            double rightStickY = -gpad.right_stick_y;   //Forward and Backward
            double leftStickX = gpad.left_stick_x;  //Turning
            double rightStickX = gpad.right_stick_x;    //Strafing
            mecDrive(leftStickY, rightStickY, leftStickX, rightStickX);

            //Latch Controls
            if (conveyorPow >= -1 && conveyorPow <= 1) {
                conveyorPow = 0;
            }
            //if (flyPow >= -1 && flyPow <= 1) {
            //    flyPow = 0;
            //}
            if (intakePow >= -1 && intakePow <= 1) {
                intakePow = 0;
            }
            if (true) {
                grabberPow = 0;
            }


            //Distance Sensor controls
            if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
                flyPow = -1600;
                conveyorPow = -1.75;
            }



            //Button Controls

            if (gpad.a) {
                robot.grabber.setPosition(0.025);
            }
            if (gpad.aShift) {
                grabberPow = 750;
            }
            if (gpad.b) {
                robot.grabber.setPosition(0.5);
            }
            if (gpad.bShift) {
                grabberPow = -750;
            }

            //Bumper and Touch Switch Controls
            double indexerPos = 0.85;
            if (gpad.right_bumper) indexerPos = 0.5;

            //D-pad Controls
            if (gpad.dpad_up && linearPos < 0.9) {
                linearPos = linearPos + 0.001;
            }
            if (gpad.dpad_down && linearPos > 0.1) {
                linearPos = linearPos - 0.001;
            }
            if (gpad.dpad_left) {
                linearPos = 0.742;
            }
            if (gpad.dpad_right) {
                linearPos = 0.672;
            }

            //Unlatch
            if (gpad.x) {
                flyPow = 0;
                conveyorPow = 0;
            }
            if (gpad.xShift) {
                flyPow = -1600;
                conveyorPow = -1.75;
            }

            //Trigger Controls
            if (gamepad1.right_trigger > 0) {
                intakePow = -1;
                if(conveyorPow != -1.75){
                    conveyorPow = -0.75;
                }
            }
            if (gamepad1.left_trigger > 0) {
                intakePow = 1;
                if(conveyorPow != -1.75){
                    conveyorPow = 0.75;
                }
            }

            //Set Powers
            robot.conveyor.setPower(motorPow(conveyorPow));
            robot.shooter1.setVelocity(flyPow);
            robot.intake.setPower(motorPow(intakePow));
            robot.wobble.setVelocity(grabberPow);
            robot.tilt.setPosition(linearPos);
            robot.indexer.setPosition(indexerPos);

            telemetry.addData("TiltPos", linearPos);
            telemetry.addData("This is new code", robot.shooter1.getVelocity());

            telemetry.update();
        }
    }

    //Method for Mecanum Drive
    public void mecDrive(double forward, double forward2, double turn, double strafe) {
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method to get motor powers
    public double motorPow(double x){
        if(x>1){
            return x-1;
        }
        if(x<-1){
            return x+1;
        }
        return x;
    }

    //Method to get servo powers
    public double servoPow(double x){
        return (motorPow(x)*0.4) + 0.5;
    }

}




