package org.firstinspires.ftc.teamcode.drive.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    RobotHardware robot = new RobotHardware();
    double turretPos = 0.07;

    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;

    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.setPose(new Pose2d(-61.5 * 0.0254, -16.5 * 0.0254, Rotation2d.fromDegrees(0)));
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
        robot.indexer.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
        robot.turret.setPosition(turretPos);

    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

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
        if (intakePow >= -1 && intakePow <= 1) {
            intakePow = 0;
        }


        //Distance Sensor controls
        if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
            flyPow = -1600;
            conveyorPow = -2;
        }

        //Bumper and Touch Switch Controls
        double indexerPos = 0.5;
        if (gpad.right_bumper) indexerPos = 0.0;

        //Unlatch
        if (gpad.x) {
            flyPow = 0;
            conveyorPow = 0;
        }
        if (gpad.xShift) {
            flyPow = -1600;
            conveyorPow = -2;
        }

        if(gpad.dpad_right && turretPos > 0.02){
            turretPos -= 0.005;
        }
        if(gpad.dpad_left && turretPos < 0.7){
            turretPos += 0.005;
        }

        //Trigger Controls
        if (gamepad1.right_trigger > 0) {
            intakePow = -1;
            if(conveyorPow != -2){
                conveyorPow = -1;
            }
        }
        if (gamepad1.left_trigger > 0) {
            intakePow = 1;
            if(conveyorPow != -2){
                conveyorPow = 1;
            }
        }

        if(gpad.y){
            turretPos = calcServo(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254); 
        }

        //Set Powers
        robot.conveyor.setPower(motorPow(conveyorPow));
        robot.shooter1.setVelocity(flyPow);
        robot.intake.setPower(motorPow(intakePow));
        robot.indexer.setPosition(indexerPos);
        robot.turret.setPosition(turretPos);

 

        telemetry.addData("X", up.pose.getTranslation().getX() / 0.0254);
        telemetry.addData("Y", up.pose.getTranslation().getY() / 0.0254);
        telemetry.addData("Heading", rotation.getDegrees());
        telemetry.addData("Turret", turretPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        slamra.stop();
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

    public double calcAngle(double X, double Y){
        double Opp = Math.abs(36+Y);
        double Adj = 72-X;
        double Theta = Math.toDegrees(Math.atan(Opp/Adj));
        return Theta;
    }

    public double calcServo(double X, double Y){
        double servoPos = (-0.01452*calcAngle(X,Y)) + 0.6634;
        return servoPos;
    }

}
