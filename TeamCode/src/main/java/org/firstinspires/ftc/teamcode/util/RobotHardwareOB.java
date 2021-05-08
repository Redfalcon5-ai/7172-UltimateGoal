package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    public final double INTAKE_POWER_INTAKE = 1;
    public final double INTAKE_POWER_OFF = 0;
    public final double INTAKE_POWER_OUTTAKE = -1.0;
    public final double CONVEYOR_POWER_INTAKE = 0.90;
    public final double CONVEYOR_POWER_FIRE = 0.90;
    public final double CONVEYOR_POWER_OUTTAKE = -0.75;
    public final double CONVEYOR_POWER_OFF = 0;
    public final double INDEXER_POSITION_OFF = 0.5;
    public final double INDEXER_POSITION_LOAD = 0.5;
    public final double INDEXER_POSITION_FIRE = 0;
    public final double SHOOTER_VELOCITY_NORMAL = 1600;
    public final double SHOOTER_VELOCITY_LOW = 1500;
    public final double SHOOTER_VELOCITY_OFF = 0;
    public final double GRABBER_POSITION_CLOSE = 0.065;
    public final double GRABBER_POSITION_OPEN = 0.85;
    public final double WOBBLE_VELOCITY_STOW = -700;
    public final double WOBBLE_VELOCITY_FLIP = 700;
    public final double TILT_POSITION_INIT = 0.66;
    public final double RPIXY_TARGET_VOLT = 1.95;
    public final double RPIXY_TARGET_RANGE = 0.2;
    public final double RPIXY_TARGET_OFFSET[] = { 0, 0.40, 0.65, 0.9 };
    public final double BPIXY_TARGET_VOLT = 1.936;
    public final double BPIXY_TARGET_RANGE = 0.2;
    public final double BPIXY_TARGET_OFFSET[] = { 0, -0.40, -0.65, -0.9 };

    public final double LARM_POSITION_UP = 0.4;
    public final double LARM_POSITION_DOWN = 0.75;
    public final double RARM_POSITION_UP = 0.6;
    public final double RARM_POSITION_DOWN = 0.2;

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
    public DigitalChannel magnet = null;
    public NormalizedColorSensor colorv3 = null;
    public DcMotorEx wobble = null;
    public Servo grabber = null;

    public Servo larm = null;
    public Servo rarm = null;

    public DigitalChannel ledr0 = null;
    public DigitalChannel ledr1 = null;
    public DigitalChannel ledg0 = null;
    public DigitalChannel ledg1 = null;

    public BNO055IMU imu = null;
    public BNO055IMU imu1 = null;

    public AnalogInput lrange = null;
    public double lrangeV = 0;

    public AnalogInput rpixyvolt = null;
    public AnalogInput rpixytrig = null;
    public AnalogInput bpixyvolt = null;
    public AnalogInput bpixytrig = null;
    public AnalogInput pixyvolt = null;
    public AnalogInput pixytrig = null;
    public double pixyV = 0;
    public double pixyTargetBase = RPIXY_TARGET_VOLT;
    public double pixyTargetV = RPIXY_TARGET_VOLT;
    public double pixyTargetR = RPIXY_TARGET_RANGE;
    public double[] pixyTargetVOff = RPIXY_TARGET_OFFSET;

    //Create Hardware Map Object
    HardwareMap hwMap = null;

    public double intakePower = 0.0;
    public double conveyorPower = 0.0;
    public double fireVelocity = SHOOTER_VELOCITY_NORMAL;
    public ElapsedTime intakeTimer = new ElapsedTime();

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
        // tilt = hwMap.get(Servo.class, "tilt");
        indexer = hwMap.get(Servo.class, "indexer");
        larm = hwMap.get(Servo.class, "larm");
        rarm = hwMap.get(Servo.class, "rarm");
        shooter1 = (DcMotorEx)hwMap.get(DcMotor.class, "shooter1");
        colorv3 = hwMap.get(NormalizedColorSensor.class, "colorv3");
        wobble = (DcMotorEx)hwMap.get(DcMotor.class, "wobble");
        grabber = hwMap.get(Servo.class, "grabber");

        lrange = hwMap.get(AnalogInput.class, "lrange");
        rpixyvolt = hwMap.get(AnalogInput.class, "rpixyvolt");
        rpixytrig = hwMap.get(AnalogInput.class, "rpixytrig");
        bpixyvolt = hwMap.get(AnalogInput.class, "bpixyvolt");
        bpixytrig = hwMap.get(AnalogInput.class, "bpixytrig");
        setPixyBlue();

        // Set all motors to zero power
        //Set servos to starting position
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        intake.setPower(0);
        conveyor.setPower(0);
        // tilt.setPosition(TILT_POSITION_INIT);
        indexer.setPosition(INDEXER_POSITION_LOAD);
        shooter1.setPower(0);
        wobble.setPower(0);
        grabber.setPosition(GRABBER_POSITION_OPEN);

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

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        imu1 = hwMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(params);


        ledr0 = hwMap.get(DigitalChannel.class, "led4");
        ledr0.setMode(DigitalChannel.Mode.OUTPUT);
        ledr0.setState(true);
        ledr1 = hwMap.get(DigitalChannel.class, "led6");
        ledr1.setMode(DigitalChannel.Mode.OUTPUT);
        ledr1.setState(true);

        ledg0 = hwMap.get(DigitalChannel.class, "led5");
        ledg0.setMode(DigitalChannel.Mode.OUTPUT);
        ledg0.setState(true);
        ledg1 = hwMap.get(DigitalChannel.class, "led7");
        ledg1.setMode(DigitalChannel.Mode.OUTPUT);
        ledg1.setState(true);

    }

    public void intake() {
        intakePower = INTAKE_POWER_INTAKE;
        conveyorPower = CONVEYOR_POWER_INTAKE;
        intakeTimer.reset();
        if (smode == ShootMode.IDLE)
            setShootMode(ShootMode.LOAD);
    }

    public void outtake() {
        intakePower = INTAKE_POWER_OUTTAKE;
        conveyorPower = CONVEYOR_POWER_OUTTAKE;
    }

    public void shooter(double v) {
        shooter1.setVelocity(v);
    }

    public void setFireVelocity(double v) {
        fireVelocity = v;
    }

    public void fire() {
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
        if (smode != s) {
            smode = s;
            smodeTimer.reset();
        }
    }

    public void updateAll() {
        // update pixy LED status
        double v = getPixyV();
        boolean red = true;      // default led4 (red) off
        boolean grn = true;      // default led5 (green) off
        if (v >= 0) {
            if (Math.abs(v - pixyTargetV) <= pixyTargetR) grn = false;
            else red = false;
        }
        if (ledr0.getState() != red) ledr0.setState(red);
        if (ledr1.getState() != red) ledr1.setState(red);
        if (ledg0.getState() != grn) ledg0.setState(grn);
        if (ledg1.getState() != grn) ledg1.setState(grn);

        double indexerPos = INDEXER_POSITION_OFF;
        if (smode == ShootMode.TRIGGER) {   // "fire" button requested
            conveyorPower = CONVEYOR_POWER_FIRE;
            indexerPos = INDEXER_POSITION_LOAD;
            shooter(fireVelocity);
            if (isFlyReady() || true) setShootMode(ShootMode.FIRE);
            if (smodeTimer.seconds() > 1.0)
                setShootMode(ShootMode.LOAD);
        }
        if (smode == ShootMode.FIRE) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (isRingLoaded()) setShootMode(ShootMode.RECOVER);
            if (smodeTimer.seconds() > 0.5) {
                setShootMode(ShootMode.LOAD);
            }
        }
        if (smode == ShootMode.RECOVER) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (!isRingLoaded()) setShootMode(ShootMode.LOAD);
            if (smodeTimer.seconds() > 0.3) setShootMode(ShootMode.LOAD);
        }
        if (smode == ShootMode.LOAD) {
            indexerPos = INDEXER_POSITION_LOAD;
            if (isRingLoaded()) shooter(fireVelocity);
        }

        updateWG();

        if (intakeTimer.seconds() < 1) conveyorPower = CONVEYOR_POWER_INTAKE;

        intake.setPower(motorPower(intakePower));
        conveyor.setPower(motorPower(conveyorPower));
        indexer.setPosition(indexerPos);

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
        boolean s = ((DistanceSensor)colorv3).getDistance(DistanceUnit.CM) < 4;
        // if (s != led6.getState()) led6.setState(s);
        return s;
    }

    public boolean isFlyReady() {
        double vel = shooter1.getVelocity();
        boolean s = (vel >= fireVelocity -40 && vel <= fireVelocity + 20);
        // if (s != led7.getState()) led7.setState(s);
        return s;
    }

    public boolean isShooterReady() {
        return isRingLoaded() && isFlyReady();
    }

    public double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getIMU1Heading() {
        Orientation angles = imu1.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getHeading() {
        return getIMUHeading();
    }

    public void driveYXW(double ry, double rx, double rw) {
        // ry == forward, rx == strafe, rw == turn
        lf.setPower(ry + rw + rx);
        rf.setPower(ry - rw - rx);
        lb.setPower(ry + rw - rx);
        rb.setPower(ry - rw + rx);
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

    public double getLRangeV() {
        return lrange.getVoltage();
    }

    // drive robot forward/strafe, maintain heading of th
    public void driveYXH(double ry, double rx, double th) {
        double herror = getHeading() - th;
        driveYXW(ry, rx, herror * 0.015);
    }

    // drive robot forward, maintain heading th and distance from wall dv
    public void driveYDH(double ry, double dv, double th) {
        double herror = getHeading() - th;
        double derror = dv - getLRangeV();
        if (herror < -10 || herror > 10) derror = 0;
        driveYXW(ry, derror * 25, herror * 0.02);
    }

    public void setArms(boolean down) {
        larm.setPosition(down ? LARM_POSITION_DOWN : LARM_POSITION_UP);
        rarm.setPosition(down ? RARM_POSITION_DOWN : RARM_POSITION_UP);
    }

    public void setPixyRed() {
        pixyvolt = rpixyvolt;
        pixytrig = rpixytrig;
        pixyTargetBase = RPIXY_TARGET_VOLT;
        pixyTargetV = pixyTargetBase;
        pixyTargetR = RPIXY_TARGET_RANGE;
        pixyTargetVOff = RPIXY_TARGET_OFFSET;
    }

    public void setPixyBlue() {
        pixyvolt = bpixyvolt;
        pixytrig = bpixytrig;
        pixyTargetBase = BPIXY_TARGET_VOLT;
        pixyTargetV = pixyTargetBase;
        pixyTargetR = BPIXY_TARGET_RANGE;
        pixyTargetVOff = BPIXY_TARGET_OFFSET;
    }

    public void setPixyTargetBase(double v) {
        if (v >= 0) pixyTargetBase = v;
    }
    public void setPixyTargetV() {
        setPixyTargetBase(getPixyV());
    }


    public double getPixyV() {
        pixyV = (pixytrig.getVoltage() > 1.5) ? pixyvolt.getVoltage() : -100;
        return pixyV;
    }

    // drive robot forward/strafe, maintain heading of pixy center
    public void driveYXP(double ry, double rx, int target) {
        double v = getPixyV();
        pixyTargetV = pixyTargetBase + pixyTargetVOff[target];
        double perror = (v >= 0) ? v - pixyTargetV : 0;
        double pgain = (ry != 0 || rx != 0) ? 1.2 : 0.4;
        driveYXW(ry, rx, perror * pgain);
    }

    public void driveYXP(double ry, double rx) {
        driveYXP(ry, rx, 0);
    }

}

