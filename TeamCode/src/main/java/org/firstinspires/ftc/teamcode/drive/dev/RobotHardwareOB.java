package org.firstinspires.ftc.teamcode.drive.dev;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardwareOB
{

    public final double INTAKE_POWER_IN = 1.0;
    public final double INTAKE_POWER_OFF = 0;
    public final double INTAKE_POWER_OUT = -1.0;
    public final double CONVEYOR_POWER_IN = 0.90;
    public final double CONVEYOR_POWER_FIRE = 0.75;
    public final double CONVEYOR_POWER_LOAD = 0.75;
    public final double CONVEYOR_POWER_LOADED = 0.75;
    public final double CONVEYOR_POWER_OUT = -0.75;
    public final double CONVEYOR_POWER_OFF = 0;
    public final double INDEXER_POSITION_LOAD = 0.5;
    public final double INDEXER_POSITION_FIRE = 0.6;
    public final double SHOOTER_VELOCITY_NORMAL = 1700;
    public final double SHOOTER_VELOCITY_LOW = 1400;
    public final double SHOOTER_VELOCITY_OFF = 0;
    public final double GRABBER_POSITION_CLOSE = 0.025;
    public final double GRABBER_POSITION_OPEN = 0.5;
    public final double WOBBLE_VELOCITY_STOW = -700;
    public final double WOBBLE_VELOCITY_FLIP = 700;
    public final double TILT_POSITION_INIT = 0.66;

    public enum ShootMode { IDLE, LOAD, TRIGGER, FIRE, RECOVER }
    public ShootMode smode = ShootMode.IDLE;
    public ElapsedTime smodeTimer = new ElapsedTime();

    public static enum WGMode { IDLE, STOW, FLIP }
    public WGMode wgmode = WGMode.IDLE;
    public ElapsedTime wgmodeTimer = new ElapsedTime();

    //Create variables for hardware
    public DcMotor lf   = null;
    public DcMotor rf   = null;
    public DcMotor lb   = null;
    public DcMotor rb   = null;
    public DcMotor intake = null;
    public DcMotor conveyor = null;
    public Servo tilt = null;
    public Servo indexer = null;
    public DcMotorEx shooter1 = null;
    public DigitalChannel touch = null;
    public DigitalChannel magnet = null;
    public NormalizedColorSensor colorv3 = null;
    public DcMotorEx wobble = null;
    public Servo grabber = null;

    public BNO055IMU imu = null;

    //Create Hardware Map Object
    HardwareMap hwMap = null;

    public double intakePower = 0.0;
    public double conveyorPower = 0.0;
    public double fireVelocity = SHOOTER_VELOCITY_NORMAL;

    //Initialize Hardware That
    //Comes from the Config
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotor.class, "lf");
        rf = hwMap.get(DcMotor.class, "rf");
        lb = hwMap.get(DcMotor.class, "lb");
        rb = hwMap.get(DcMotor.class, "rb");
        intake = hwMap.get(DcMotor.class, "intake");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
        tilt = hwMap.get(Servo.class, "tilt");
        indexer = hwMap.get(Servo.class, "indexer");
        shooter1 = (DcMotorEx)hwMap.get(DcMotor.class, "shooter1");
        touch = hwMap.get(DigitalChannel.class, "touch");
        magnet = hwMap.get(DigitalChannel.class, "magnet");
        colorv3 = hwMap.get(NormalizedColorSensor.class, "colorv3");
        wobble = (DcMotorEx)hwMap.get(DcMotor.class, "wobble");
        grabber = hwMap.get(Servo.class, "grabber");

        // Set all motors to zero power
        //Set servos to starting position
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        intake.setPower(0);
        conveyor.setPower(0);
        tilt.setPosition(TILT_POSITION_INIT);
        indexer.setPosition(INDEXER_POSITION_LOAD);
        shooter1.setPower(0);
        wobble.setPower(0);
        grabber.setPosition(0.5);

        // Set all motors to run without encoders
        //Set mode for sensors
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocityPIDFCoefficients(150, 0, 0, 13);
        wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        touch.setMode(DigitalChannel.Mode.INPUT);
        magnet.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }

    public void intake() {
        intakePower = INTAKE_POWER_IN;
        conveyorPower = CONVEYOR_POWER_IN;
        if (smode == ShootMode.IDLE)
            setShootMode(ShootMode.LOAD);
    }

    public void outtake() {
        intakePower = INTAKE_POWER_OUT;
        conveyorPower = CONVEYOR_POWER_OUT;
    }

    public void shooter(double v) {
        shooter1.setVelocity(v);
    }

    public void setFireVelocity(double v) {
        fireVelocity = v;
    }

    public void fire() {
        if (smode == ShootMode.LOAD)
            setShootMode(ShootMode.TRIGGER);
    }

    public void quiet() {
        intakePower = INTAKE_POWER_OFF;
        conveyorPower = CONVEYOR_POWER_OFF;
        shooter(SHOOTER_VELOCITY_OFF);
        indexer.setPosition(INDEXER_POSITION_LOAD);
        setShootMode(ShootMode.IDLE);
    }

    public void setShootMode(ShootMode s) {
        smode = s;
        smodeTimer.reset();
    }

    public void updateAll() {
        if (smode == ShootMode.LOAD) {
            if (isRingLoaded()) shooter(fireVelocity);
            if (conveyorPower == 0) {  // intake can override this
                conveyorPower = isRingLoaded()
                        ? CONVEYOR_POWER_LOADED
                        : CONVEYOR_POWER_LOAD;
            }
        }
        if (smode == ShootMode.TRIGGER) {
            conveyorPower = CONVEYOR_POWER_FIRE;
            if(smodeTimer.seconds() > 2){
                smode = ShootMode.LOAD;
            }
            else if (isShooterReady()) {
                indexer.setPosition(INDEXER_POSITION_FIRE);
                setShootMode(ShootMode.FIRE);
            }
        }
        if (smode == ShootMode.FIRE) {
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (!isRingLoaded() || smodeTimer.seconds() > 0.60) {
                indexer.setPosition(INDEXER_POSITION_LOAD);
                setShootMode(ShootMode.RECOVER);
            }
        }
        if (smode == ShootMode.RECOVER) {
            if (smodeTimer.seconds() > 0.1)
                setShootMode(ShootMode.LOAD);
        }

        updateWG();

        intake.setPower(motorPower(intakePower));
        conveyor.setPower(motorPower(conveyorPower));

        intakePower = delatch(intakePower);
        conveyorPower = delatch(conveyorPower);
    }

    public double motorPower(double x) {
        // convert power values in 1.0..2.0 (latched) to be in 0..1.0.
        if (x > 1.0) return x-1;
        if (x < -1.0) return x+1;
        return x;
    }

    public double delatch(double x) {
        // keep x if it's latched, set to zero otherwise
        if (x < -1.0 || x > 1.0) return x;
        return 0;
    }

    public boolean isRingLoaded() {
        return ((DistanceSensor)colorv3).getDistance(DistanceUnit.CM) < 4;
    }

    public boolean isShooterReady() {
        double vel = shooter1.getVelocity();
        return isRingLoaded()
                && vel >= fireVelocity - 20
                && vel <= fireVelocity + 20;
    }

    public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void driveYXW(double ry, double rx, double rw) {
        // ry == forward, rx == strafe, rw == turn
        lf.setPower(ry + rw + rx);
        rf.setPower(ry - rw - rx);
        lb.setPower(ry + rw - rx);
        rb.setPower(ry - rw + rx);
    }

    public void driveYXH(double ry, double rx, double th) {
        double h = getIMUHeading() - th;
        driveYXW(ry, rx, h * 0.02);
    }

    public void wgStow() { setWGMode(WGMode.STOW); }

    public void wgFlip() { setWGMode(WGMode.FLIP); }

    public void wgOpen() {
        grabber.setPosition(GRABBER_POSITION_OPEN);
    }

    public void wgClose() {
        grabber.setPosition(GRABBER_POSITION_CLOSE);
    }

    public void setWGMode(WGMode wgm) {
        wgmode = wgm;
        wgmodeTimer.reset();
    }

    public void updateWG() {
        double wobbleVelocity = 0;
        if (wgmode == WGMode.STOW) {
            if (wgmodeTimer.seconds() < 1.5)
                wobbleVelocity = WOBBLE_VELOCITY_STOW;
        }
        if (wgmode == WGMode.FLIP) {
            if (wgmodeTimer.seconds() < 1.5)
                wobbleVelocity = WOBBLE_VELOCITY_FLIP;
        }
        wobble.setVelocity(wobbleVelocity);
    }

}

