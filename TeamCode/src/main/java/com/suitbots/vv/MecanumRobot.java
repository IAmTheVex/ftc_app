package com.suitbots.vv;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.RunnableFuture;

/**
 * Created by Suit Bots on 11/11/2016.
 */

public class MecanumRobot {
    private DcMotor lf, lr, rf, rr, harvester, flipper;
    private ToggleableServo dispenser;
    private LazyCR pf, pr;
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor color;
    private Telemetry telemetry;
    private DeviceInterfaceModule dim;
    private OpticalDistanceSensor ods;
    private TouchSensor touch;


    public MecanumRobot(HardwareMap hardwareMap, Telemetry _telemetry) {
        telemetry = _telemetry;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");
        flipper = hardwareMap.dcMotor.get("flipper");
        harvester = hardwareMap.dcMotor.get("harvester");
        touch = hardwareMap.touchSensor.get("touch");

        pf = new LazyCR(hardwareMap.crservo.get("pf"));
        pr = new LazyCR(hardwareMap.crservo.get("pr"));
        dispenser = new ToggleableServo(hardwareMap.servo.get("dispenser"), 0.0, .3);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);
        ods = hardwareMap.opticalDistanceSensor.get("line");

        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        gyro.calibrate();

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void onStart() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rr, rf, flipper);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf, flipper);
        dispenser.set(0.0);
        dispenser.onStart();
        pf.setPower(0.0);
        pr.setPower(0.0);
        flipper.setPower(0.0);
        flipping = false;
    }

    public void onStop() {
        stopDriveMotors();
        flipper.setPower(0.0);
        harvester.setPower(0.0);
        pf.setPower(0.0);
        pr.setPower(0.0);
        dispenser.set(0.0);
        flipping = false;
    }

    private interface Stoppable {
        public boolean stopped();
    }

    private class ButtonPresser implements Stoppable {
        private LazyCR s;
        private long t0;
        public ButtonPresser(LazyCR s_) {
            s = s_;
            t0 = System.currentTimeMillis();
        }

        @Override
        public boolean stopped() {
            if(1000 > (System.currentTimeMillis() - t0)) {
                return false;
            }
            s.setPower(0.0);
            return true;
        }
    }

    private ArrayList<Stoppable> stoppables = new ArrayList<>();

    private int averageRemainingTicks(DcMotor... ms) {
        int total = 0;
        int count = 0;
        for (DcMotor m : ms) {
            if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 100 < Math.abs(m.getTargetPosition())) {
                total += Math.abs(m.getTargetPosition() - m.getCurrentPosition());
                count += 1;
            }
        }
        return 0 == count ? 0 : total / count;
    }

    private static int SLOW_DOWN_HERE = 1120;
    private static double ARBITRARY_SLOW_SPEED = .3;
    private boolean slowedDown = false;
    private void encoderDriveSlowdown() {
        if (! slowedDown) {
            if (lf.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                int remaining = averageRemainingTicks(lf, lr, rf, rr);
                if (remaining < SLOW_DOWN_HERE) {
                    slowedDown = true;
                    setPower(ARBITRARY_SLOW_SPEED, lf, lr, rf, rr);
                }
            }
        }
    }

    // Things that need to happen in the teleop loop to accommodate long-running
    // tasks like running the flipper one at a time.
    public void loop() {
        if (isDoneFlipping()) {
            setFlipperPower(0.0);
        }

        if (collect_light_meter_info) {
            total_light_meter_reading += ods.getRawLightDetected();
            light_meter_readings_tooken++;
        }


        encoderDriveSlowdown();
        // manageEncoderAccelleration(lf, lr, rf, rr);
    }

    private class Stopper implements Runnable {
        private final LazyCR servo;
        private final long time;
        public Stopper(LazyCR s, long _time) {
            servo = s;
            time = _time;
        }
        @Override
        public void run() {
            try {
                Thread.sleep(1000);
            } catch(InterruptedException ie) {
                // pass
            }
            servo.setPower(0.0);
        }
    }

    private static final long MAX_PRESS_TIME = 1000;
    private void pressButton(final LazyCR servo) throws InterruptedException {
        servo.setPower(-1.0);
        final long t0 = System.currentTimeMillis();
        boolean activated = false;
        while (MAX_PRESS_TIME > (System.currentTimeMillis() - t0)) {
            Thread.sleep(10);
        }
        final long t1 = System.currentTimeMillis();
        if(! activated) {
            servo.setPower(0.0);
            Thread.sleep(250);
        }
        servo.setPower(1.0);

        Thread th = new Thread(new Stopper(servo, t0 - t1));
        th.start();
    }

    public void pressFrontButton() throws InterruptedException {
        pressButton(pf);
    }

    public void pressBackButton() throws InterruptedException {
        pressButton(pr);
    }

    public void updateSensorTelemetry() {
        telemetry.addData("Flipping", flipping ? "Yes" : "No");
        telemetry.addData("Gyro",  gyro.getIntegratedZValue());
        telemetry.addData("Color", String.format(Locale.US, "R: %d\tB: %d", color.red(), color.blue()));
        telemetry.addData("Light", getLineLightReading());
        telemetry.addData("Encoder Remain", averageRemainingTicks(lf, lr, rf, rr));
        telemetry.addData("EncodersC", String.format(Locale.US, "%d\t%d\t%d\t%d\t%d",
                lf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rf.getCurrentPosition(),
                rr.getCurrentPosition(),
                flipper.getCurrentPosition()));
        telemetry.addData("EncodersT", String.format(Locale.US, "%d\t%d\t%d\t%d\t%d",
                lf.getTargetPosition(),
                lr.getTargetPosition(),
                rf.getTargetPosition(),
                rr.getTargetPosition(),
                flipper.getTargetPosition()));
    }

    // The Flipper
    public static final int ONE_FILPPER_REVOLUTION = (1120 * 22) / 16;
    private boolean flipping = false;
    public void fire() {
        if (flipping) {
            return;
        }
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (0 != flipper.getCurrentPosition()) {
            throw new RuntimeException("Well there's your problem (Bad encoder value)");
        }
        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setTargetPosition(ONE_FILPPER_REVOLUTION);
        flipper.setPower(1.0);
        if (! flipper.isBusy()) {
            throw new RuntimeException("Flipper should be busy.");
        }
        flipping = true;
    }

    public boolean isFlipping() { return flipping; }

    public static final int FLIPPER_CLOSE_ENOUGH = 2;
    private boolean isDoneFlipping() {
        if (! flipping) {
            return false;
        }

        if (flipper.getMode() == DcMotor.RunMode.RUN_TO_POSITION && ! flipper.isBusy()) {
            return true;
        }

        return false;
    }

    public void stopFlipperIfItIsNotFlipping() {
        if (! flipping) {
            flipper.setPower(0.0);
        }
    }

    public void setFlipperPower(double p) {
        if (flipping) {
            flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flipping = false;
        }
        flipper.setPower(p);
    }

    // Harvesting

    public void setHarvesterPower(double p) {
        harvester.setPower(p);
    }

    // Servo Control

    public void setFrontPower(double p) { pf.setPower(p); }
    public void setBackPower(double p) { pr.setPower(p); }
    public void toggleDispenser() {
        dispenser.toggle();
    }
    public void setDispenser(boolean x) {
        dispenser.setFirst(x);
    }

    // Sensors

    public boolean touchSensorPressed() {
        return touch.isPressed();
    }

    public double getLineLightReading() {
        return ods.getRawLightDetected();
    }

    public boolean isCalibrating(){  return gyro.isCalibrating(); }
    public void resetGyro() {
        gyro.resetZAxisIntegrator();
    }
    public int getHeading() {
        int angle = gyro.getIntegratedZValue() % 360;
        return angle;
    }

    public static final int COLOR_THRESHOLD = 2;
    public boolean colorSensorIsRed() {
        return color.red() > COLOR_THRESHOLD && color.red() > color.blue();
    }

    public AllianceColor getColor() {
        AllianceColor c = color.red() > color.blue() ? AllianceColor.RED : AllianceColor.BLUE;
        return c;
    }

    public boolean colorSensorIsBlue() {
        return color.blue() > COLOR_THRESHOLD && color.blue() > color.red();
    }

    public int getColorAlpha() {
        return color.alpha();
    }

    // Driving

    public void drivePreservingDirection(double translationRadians, double velocity) {
        final int angle = gyro.getIntegratedZValue();
        if (0 != angle) {
            final double rotSpeed = Math.log((double) Math.abs(angle)) * (angle < 0 ? -1.0 : 1.0);
            drive(translationRadians, velocity, rotSpeed);
        }
    }

    /// Maximum absolute value of some number of arguments
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    private static class Wheels {
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }

    private Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s =  Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    public void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lr.setPower(w.lr);
        rr.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "%.2f %.2f %.2f %.2f", w.lf, w.rf, w.lr, w.rr));
    }

    /// Shut down all motors
    public void stopDriveMotors() {
        lf.setPower(0.0);
        lr.setPower(0.0);

        rf.setPower(0.0);
        rr.setPower(0.0);
    }

    // Encoder Driving

    // Assuming 4" wheels
    private static final double TICKS_PER_INCH = 1120 * (16./24.) / (Math.PI * 4.0);
    private static final double TICKS_PER_CM = TICKS_PER_INCH / 2.54;
    private static final double ENCODER_DRIVE_POWER = .3; // .35;

    private double encoder_drive_power = ENCODER_DRIVE_POWER;

    void setEncoderDrivePower(double p) {
        encoder_drive_power = p;
    }

    void clearEncoderDrivePower() {
        encoder_drive_power = ENCODER_DRIVE_POWER;
    }

    private void setMode(DcMotor.RunMode mode, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setMode(mode);
        }
    }

    private void setPower(double p, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setPower(p);
        }
    }

    private void setTargetPosition(int pos, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setTargetPosition(pos);
        }
    }

    public static final int ENCODERS_CLOSE_ENOUGH = 10;
    private boolean busy(DcMotor... ms) {
        int total = 0;
        for (DcMotor m : ms) {
            if (m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total += Math.max(0, t - c);
            }
        }
        return total > ENCODERS_CLOSE_ENOUGH;
    }

    public boolean driveMotorsBusy() {
        return busy(lf, lr, rf, rr);

    }

    public void encoderDriveTiles(double direction, double tiles) {
        encoderDriveInches(direction, 24.0 * tiles);
    }

    public void encoderDriveInches(double direction, double inches) {
        final Wheels w = getWheels(direction, 1.0, 0.0);
        final int ticks = (int)(inches * TICKS_PER_INCH);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDriveCM(double direction, double cm) {
        direction %= Math.PI * 2.0;
        final Wheels w = getWheels(direction, 1.0, 0.0);
        final int ticks = (int)(cm * TICKS_PER_CM);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    private void encoderDrive(double lft, double rft, double lrt, double rrt) {
        encoderDrive((int) lft, (int) rft, (int) lrt, (int) rrt);
    }

    private void encoderDrive(int lft, int rft, int lrt, int rrt) {
        setPower(0.0, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setTargetPosition(lft, lf);
        setTargetPosition(rft, rf);
        setTargetPosition(lrt, lr);
        setTargetPosition(rrt, rr);
        setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);
        setPower(encoder_drive_power, lf, lr, rf, rr);
        slowedDown = false;
    }


    // All motors start out at ENCODER_DRIVE_POWER power. Once we get one revolution
    // in, go ahead and speed up. When we get within a revolution of the end of our
    // run, start slowing down. The idea here is to avoid slip.
    private static final int ACCEL_THRESHOLD = 1120 * 24 / 18; // one wheel revolution, for starters
    private boolean atSteadyState = false;
    private void manageEncoderAccelleration(DcMotor... ms) {
        if (encoder_drive_power > ENCODER_DRIVE_POWER) {
            int current = 0, remaining = 0, count = 0;

            ArrayList<DcMotor> driving = new ArrayList<>();
            for (DcMotor m : ms) {
                if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 0 != m.getTargetPosition()) {
                    driving.add(m);
                    current += m.getCurrentPosition();
                    remaining += Math.abs(m.getCurrentPosition() - m.getTargetPosition());
                    count++;
                }
            }

            if (0 < driving.size()) {
                current /= count;
                remaining /= count;

                double power = encoder_drive_power;
                double dp = encoder_drive_power - ENCODER_DRIVE_POWER;

                if (remaining < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)remaining / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                } else if (current < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)current / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                }  else {
                    if (! atSteadyState) {
                        for (DcMotor m : driving) {
                            m.setPower(power);
                        }
                        atSteadyState = true;
                    }
                }

            }
        }
    }



    public void resetDriveMotorModes() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    private double total_light_meter_reading = 0.0;
    private int light_meter_readings_tooken = 0;
    private boolean collect_light_meter_info = false;

    public void startCollectingLightMeter() {
        collect_light_meter_info = true;
        total_light_meter_reading = 0.0;
        light_meter_readings_tooken = 0;
    }
    public void stopCollectingLightMeter() {
        collect_light_meter_info = false;
    }
    public double getAverageLightMeter() {
        return total_light_meter_reading / light_meter_readings_tooken;
    }

    public void disableEncoders() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }
}

