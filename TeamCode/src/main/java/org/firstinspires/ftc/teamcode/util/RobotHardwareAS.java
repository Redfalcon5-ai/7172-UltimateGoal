package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.RobotHardwareOB;

public class RobotHardwareAS extends RobotHardwareOB
{

    public void driveYXW(double ry, double rx, double rw) {
        // ry == forward, rx == strafe, rw == turn
        lf.setPower(ry + rw + rx);
        rf.setPower(ry - rw - rx);
        lb.setPower(ry + rw - rx);
        rb.setPower(ry - rw + rx);
    }

    public void driveYXH(double ry, double rx, double th, double ch) {
        double h = ch - th;
        driveYXW(ry, rx, h * 0.02);
    }

    public void updateAll(int velocity) {
        double indexerPos = INDEXER_POSITION_OFF;
        if (smode == ShootMode.TRIGGER) {   // "fire" button requested
            conveyorPower = CONVEYOR_POWER_FIRE;
            indexerPos = INDEXER_POSITION_LOAD;
            shooter(velocity);
            if (isShooterReady()) setShootMode(ShootMode.FIRE);
            if (smodeTimer.seconds() > 1.0)
                setShootMode(ShootMode.LOAD);
        }
        if (smode == ShootMode.FIRE) {
            indexerPos = INDEXER_POSITION_FIRE;
            conveyorPower = CONVEYOR_POWER_FIRE;
            if (smodeTimer.seconds() > 0.2) {
                setShootMode(ShootMode.LOAD);
            }
        }
        if (smode == ShootMode.LOAD) {
            indexerPos = INDEXER_POSITION_LOAD;
            if (isRingLoaded()) shooter(velocity);
        }

        updateWG();

        if (intakeTimer.seconds() < 1) conveyorPower = CONVEYOR_POWER_INTAKE;

        intake.setPower(motorPower(intakePower));
        conveyor.setPower(motorPower(conveyorPower));
        indexer.setPosition(indexerPos);

        intakePower = delatch(intakePower);
        conveyorPower = delatch(conveyorPower);
    }

    public boolean isShooterReady(int velocity) {
        double vel = shooter1.getVelocity();
        return isRingLoaded()
                && vel >= velocity - 20
                && vel <= velocity + 20;
    }



    public void driveYDH(double ry, double dv, double th, double heading) {
        double herror = heading - th;
        double derror = dv - getLRangeV();
        if (herror < -10 || herror > 10) derror = 0;
        driveYXW(ry, derror * 25, herror * 0.02);
    }


}

