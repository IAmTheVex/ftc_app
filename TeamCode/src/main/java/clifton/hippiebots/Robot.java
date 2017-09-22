package clifton.hippiebots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by pflores on 9/19/17.
 */

public class Robot {
    private DcMotor lf, rf, lr, rr;

    public Robot(HardwareMap h) {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static double maxAbs(double... ds) {
        double ret = 0.0;
        for (double d : ds) {
            ret = Math.max(Math.abs(d), ret);
        }
        return ret;
    }

    public void setMotors(double lf, double lr, double rf, double rr) {
        final double scale = maxAbs(1.0, lf, lr, rf, rr);
        this.lf.setPower(lf / scale);
        this.lr.setPower(lr / scale);
        this.rf.setPower(rf / scale);
        this.rr.setPower(rr / scale);
    }

    public void stop() {
        lf.setPower(0.0);
        lr.setPower(0.0);
        rf.setPower(0.0);
        rr.setPower(0.0);
    }
}
