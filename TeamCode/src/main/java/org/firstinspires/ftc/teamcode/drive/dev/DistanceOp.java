package org.firstinspires.ftc.teamcode.drive.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class DistanceOp extends LinearOpMode {

    AnalogInput dist = null;
    BNO055IMU imu = null;
    DcMotor lf = null;
    DcMotor rf = null;
    DcMotor lb = null;
    DcMotor rb = null;

    DigitalChannel disttrig = null;
    ElapsedTime dTimer = new ElapsedTime();
    double distVoltage = 0;
    double xtime;
    double ytime;

    @Override
    public void runOpMode() {

        dist = hardwareMap.get(AnalogInput.class, "dist0");
        disttrig = hardwareMap.get(DigitalChannel.class, "disttrig");
        disttrig.setMode(DigitalChannel.Mode.OUTPUT);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        lf = hardwareMap.get(DcMotor.class, "lf");
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf = hardwareMap.get(DcMotor.class, "rf");
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb = hardwareMap.get(DcMotor.class, "lb");
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb = hardwareMap.get(DcMotor.class, "rb");
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double baseHead = getIMUHeading();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double jy = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double jx = gamepad1.right_stick_x;
            double jw = gamepad1.left_stick_x;
            if (gamepad1.left_bumper) {
                driveYDH(jy, 0.5, baseHead);
            }
            else driveYXW(jy, jx, jw);
            
            telemetry.addData("Status", "Running");
            telemetry.addData("distV", getDistanceVoltage());
            telemetry.addData("xtime", xtime);
            telemetry.addData("ytime", ytime);
            telemetry.addData("heading", getIMUHeading());
            telemetry.update();

        }
    }
    
    double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    
    void driveYXW(double ry, double rx, double rw) {
        lf.setPower(ry+rx+rw);
        rf.setPower(ry-rx-rw);
        lb.setPower(ry-rx+rw);
        rb.setPower(ry+rx-rw);
    }
    
    void driveYDH(double ry, double rd, double rh) {
        double herror = getIMUHeading() -rh ;
        double derror = (Math.abs(herror) < 20) ? getDistanceVoltage() - rd : 0;
        driveYXW(ry, derror * 2.5, herror * 0.02);
    }
    
    double getDistanceVoltage() {
        xtime = dTimer.seconds();
        if (xtime > 0.100) {
            disttrig.setState(true);
            dTimer.reset();
        } else if (xtime > 0.01 && disttrig.getState()) {
            ytime = xtime;
            distVoltage = dist.getVoltage();
            disttrig.setState(false);
        }
        return distVoltage;
    }
}
