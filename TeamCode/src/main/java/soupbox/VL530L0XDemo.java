package soupbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import soupbox.sensor.VL530L0X;

@TeleOp(name = "VL530L0X Demo")
public class VL530L0XDemo extends OpMode {
    private VL530L0X vl530l0x;

    @Override
    public void init() {
        vl530l0x = new VL530L0X(hardwareMap.i2cDevice.get("vl530l0x"));
        if (! vl530l0x.init()) {
            throw new RuntimeException("Could not inititalize VL530L0X");
        }
    }

    @Override
    public void loop() {
        try {
            telemetry.addData("Distance", vl530l0x.readRangeSingleMillimeters());
        } catch (VL530L0X.TimeoutException te) {
            telemetry.addData("Distance", "TIMEOUT");
        }
        telemetry.update();
    }
}
