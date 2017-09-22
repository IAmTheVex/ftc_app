package clifton.hippiebots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Teleop")
public class MecanumTeleop extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        final double lx = Math.pow(gamepad1.left_stick_x, 3.0);
        final double ly = Math.pow(gamepad1.left_stick_y, 3.0);
        final double r = Math.pow(gamepad1.right_stick_x, 3.0);

        final double θ = Math.atan2(lx, ly);
        final double vθ = Math.sqrt(lx * lx + ly * ly);

        final double lf = vθ * Math.sin(θ + Math.PI / 4.0) + r;
        final double rf = vθ * Math.cos(θ + Math.PI / 4.0) - r;
        final double lr = vθ * Math.cos(θ + Math.PI / 4.0) + r;
        final double rr = vθ * Math.sin(θ + Math.PI / 4.0) - r;

        robot.setMotors(lf, lr, rf, rr);
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }
}
