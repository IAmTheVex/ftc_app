package com.suitbots.vv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

public abstract class AutonomousBase extends LinearOpMode {
    protected TankRobot robot = new TankRobot();

    /// Set up the robot's hardware
    protected void initialize() throws InterruptedException {
        robot.initHardware(hardwareMap);
        robot.calibrateGyro();
        int count = 0;
        while(robot.isGyroCalibrating()) {
            telemetry.addData("Gyro", String.format(Locale.US, "Calibrating (%d)", count++));
            telemetry.update();
            Thread.sleep(500, 0);
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.update();
    }

    private void sleep() throws InterruptedException {
        // idle();
        Thread.sleep(50, 0);
    }

    public static final int TICKS_PER_REV = 1140;
    /// Drive forward a certain number of inches using encoders
    public void fwd(double inches) throws InterruptedException {
        final int ticks = (int)(TICKS_PER_REV * inches / robot.WHEEL_RADIUS_IN);
        robot.resetDriveEncoders();
        robot.pushRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorsEncoderTarget(ticks);
        robot.setDriveMotors(.5, .5);
        while (opModeIsActive() && robot.motorsAreBusy()) {
            sleep();
        }
        robot.setDriveMotors(0.0, 0.0);
        robot.popRunMode();
    }

    /// Drive forward a certain number of field squares
    public void fwdSquares(double squares) throws InterruptedException {
        fwd(24.0 * squares);
    }

    /// Drive forward a certain number of meters
    public void fwdMeters(double meters) throws InterruptedException {
        fwd(39.3701 * meters);
    }

    /// Turn a certain number of degrees
    public void turn(int degrees) throws InterruptedException {
        turn(degrees, true);
    }

    /// Turn a certain number of degrees, optionally not resetting the gyro
    public void turn(int _degrees, boolean reset_heading) throws InterruptedException {
        final double P = .1;
        robot.pushRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (reset_heading) {
            robot.resetGyro();
        }
        if (0 < _degrees) {
            robot.setDriveMotors(P, -P);
        } else {
            robot.setDriveMotors(-P, P);
        }
        final int degrees = Math.abs(_degrees) % 360;
        while (opModeIsActive() && Math.abs(robot.getHeading()) < Math.abs(degrees)) {
            sleep();
        }
        robot.setDriveMotors(0.0, 0.0);
        robot.popRunMode();
    }

    public static final double LINE_SPEED = .4;
    public static final double LINE_THRESHOLD = 3.0;
    /// Drive forward slowly until the optical distance sensor is over a white line
    public void driveToWhiteLine() throws InterruptedException {
        robot.pushRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDriveMotors(LINE_SPEED, LINE_SPEED);
        while (opModeIsActive() && LINE_THRESHOLD > robot.getLineReading()) {
            sleep();
        }
        robot.setDriveMotors(0.0, 0.0);
        robot.popRunMode();
    }

    public static double WALL_SPEED = .2;
    public static final double WALL_DISTANCE_THRESHOLD = 10.0;
    public static double WALL_LIGHT_THRESHOLD = 50.0;
    /// Drive forward slowly until you're a certain distance from the wall
    public void driveToWall() throws InterruptedException {
        robot.pushRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDriveMotors(WALL_SPEED, WALL_SPEED);
        while (opModeIsActive() &&
                (WALL_DISTANCE_THRESHOLD < robot.getAcousticRangeCM()
                        || WALL_LIGHT_THRESHOLD < robot.getRangeLightDetected())) {
            idle();
        }
        robot.setDriveMotors(0.0, 0.0);
        robot.popRunMode();
    }
}
