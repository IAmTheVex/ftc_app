package com.suitbots.vv;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MecanumRobot {
    private DcMotor lf, rf, lr, rr;
    private ModernRoboticsI2cGyro gyro;
    private ModernRoboticsI2cRangeSensor range;
    private OpticalDistanceSensor line;

    public void initializeHardware(HardwareMap hardwareMap) throws InterruptedException {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        range = (ModernRoboticsI2cRangeSensor) hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        line = hardwareMap.opticalDistanceSensor.get("line");
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        lf.setMode(mode);
        rf.setMode(mode);
        lr.setMode(mode);
        rr.setMode(mode);
    }

    public double getAcousticRangeIN() {
        return range.getDistance(DistanceUnit.INCH);
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    public boolean isGyroCalibrating() {
        return gyro.isCalibrating();
    }

    public int getHeading() {
        return gyro.getIntegratedZValue();
    }

    public void resetGyro() {
        gyro.resetZAxisIntegrator();
    }

    public double getLineReading() {
        return line.getRawLightDetected();
    }

    public void stop() {
        lf.setPower(0.0);
        rf.setPower(0.0);
        lr.setPower(0.0);
        rr.setPower(0.0);
    }

    public static final double PI_4 = Math.PI / 4.0;
    public void drive(double translationRadians, double speed, double rotSpeedClockwise) {
        double lfp = speed * Math.sin(translationRadians + PI_4) + rotSpeedClockwise;
        double rfp = speed * Math.cos(translationRadians + PI_4) - rotSpeedClockwise;
        double lrp = speed * Math.cos(translationRadians + PI_4) + rotSpeedClockwise;
        double rrp = speed * Math.sin(translationRadians + PI_4) - rotSpeedClockwise;

        double max = Math.max(Math.abs(lfp), Math.max(Math.abs(rfp), Math.max(Math.abs(lrp), Math.abs(rrp))));

        lf.setPower(lfp / max);
        rf.setPower(rfp / max);
        lr.setPower(lrp / max);
        rr.setPower(rrp / max);
    }

    /// Controls how aggressively we correct for being off heading
    public static final double ROT_SCALE = 10.0;
    public void driveAndMaintainZeroHeading(double translationRadians, double speed) {
        double rot = - getHeading() / ROT_SCALE;
        drive(translationRadians, speed, rot);
    }

    public static double DRIVE_SPEED = .8;
    public void driveTicks(int ticks) {
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setPower(DRIVE_SPEED);
        rf.setPower(DRIVE_SPEED);
        lr.setPower(DRIVE_SPEED);
        rr.setPower(DRIVE_SPEED);
    }

    public static final int TICKS_PER_INCH = (int)(1140 / (Math.PI * 4.0));

    public boolean motorsAreBusy() {
        return lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy();
    }
}
