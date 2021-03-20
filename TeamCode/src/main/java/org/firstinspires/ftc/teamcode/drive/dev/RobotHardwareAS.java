package org.firstinspires.ftc.teamcode.drive.dev;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Templates.T265DriveSample;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.spartronics4915.lib.T265Camera;

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


}

