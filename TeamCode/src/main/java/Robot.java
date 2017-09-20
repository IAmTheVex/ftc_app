import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by pflores on 9/19/17.
 */

public class Robot {
    private DcMotor lf, rf, lb, rb;
    private double lastG;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private ColorSensor redVsBlue;
    public Robot(HardwareMap h) {
        imu = h.get(BNO055IMU.class, "gyro");
        initilizeGyro();
        redVsBlue = h.colorSensor.get("redVsBlue");

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
    private void initilizeGyro() {

    }
}
