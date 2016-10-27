package com.suitbots.vv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class ShalfBeacons extends LinearOpMode {
    private MecanumRobot robot = new MecanumRobot();

    public enum Alliance { RED, BLUE };

    public abstract Alliance alliance();
    public abstract double forward();

    public double towardsWall() { return - Math.PI / 2.0; };

    public static class Red extends ShalfBeacons {
        public Alliance alliance() { return Alliance.RED; }
        public double forward() { return 0.0; }
    }

    public static class Blue extends ShalfBeacons {
        public Alliance alliance() { return Alliance.BLUE; }
        public double forward() { return Math.PI; }
    }

    public void runOpMode() throws InterruptedException {
        robot.initializeHardware(hardwareMap);

        telemetry.addData("Gyro", "Calibrating");
        telemetry.update();
        robot.calibrateGyro();
        while (robot.isGyroCalibrating()) {
            Thread.sleep(100, 0);
        }
        telemetry.addData("Gyro", "Calibrated. Press Start");
        telemetry.update();

        robot.resetGyro();

        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean passed_line = driveToWall();
        driveToBeacon(passed_line);
        pressBeacon();
        scootForward();
        driveToBeacon(false);
        pressBeacon();
    }

    protected void allianceDrive(double radians, double speed) {
        robot.driveAndMaintainZeroHeading(radians, speed);
    }

    public static final double FAST_SPEED = 1.0;
    public static final double SLOW_SPEED = 0.4;

    public static final double LONG_WALL_DISTANCE_IN = 20.0;
    public static final double WALL_DISTANCE_IN = 10.0;
    protected boolean driveToWall() throws InterruptedException {
        double best_line = 0.0;
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(opModeIsActive() && LONG_WALL_DISTANCE_IN < robot.getAcousticRangeIN()) {
            allianceDrive(towardsWall(), FAST_SPEED);
            best_line = Math.max(best_line, robot.getLineReading());
        }
        while(opModeIsActive() && WALL_DISTANCE_IN < robot.getAcousticRangeIN()) {
            allianceDrive(towardsWall(), SLOW_SPEED);
            best_line = Math.max(best_line, robot.getLineReading());
        }
        robot.stop();
        return best_line >= LINE_THRESHOLD;
    }

    public static final double LINE_THRESHOLD = 3.0;
    protected void driveToBeacon(boolean passed_line) throws InterruptedException {
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(opModeIsActive() && LINE_THRESHOLD < robot.getLineReading()) {
            double a = forward();
            if ((WALL_DISTANCE_IN * .9) > robot.getHeading()) {
                a = - towardsWall();
            } else if ((WALL_DISTANCE_IN * 1.1) < robot.getHeading()) {
                a = towardsWall();
            }
            allianceDrive(a, SLOW_SPEED);
        }
        robot.stop();
    }

    public void scootForward() throws InterruptedException {
        robot.driveTicks(robot.TICKS_PER_INCH * 24);
        while(opModeIsActive() && robot.motorsAreBusy()) {
            idle();
        }
        robot.stop();
    }

    protected void pressBeacon() throws InterruptedException {

    }
}
