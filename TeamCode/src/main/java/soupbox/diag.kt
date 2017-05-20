package soupbox

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Motor Test")
class MotorsTest : OpMode() {
    var lf : DcMotor? = null
    var lr : DcMotor? = null
    var rf : DcMotor? = null
    var rr : DcMotor? = null

    override fun init() {
        lf = hardwareMap.dcMotor.get("lf")
        lr = hardwareMap.dcMotor.get("lr")
        rf = hardwareMap.dcMotor.get("rf")
        rr = hardwareMap.dcMotor.get("rr")
        lf?.direction = DcMotorSimple.Direction.REVERSE
        lr?.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        val power = (gamepad1.left_trigger - gamepad1.right_trigger).toDouble()
        lf?.power = if (gamepad1.x) power else 0.0
        lr?.power = if (gamepad1.y) power else 0.0
        rf?.power = if (gamepad1.a) power else 0.0
        rr?.power = if (gamepad1.b) power else 0.0
    }
}