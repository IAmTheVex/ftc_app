package soupbox

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor


val epsilon = 0.0001

fun maximum(vararg xs : Double) : Double {
    return Math.max(epsilon, xs.reduce { acc, d -> Math.max(acc, Math.abs(d)) })
}

fun cube(x : Float) : Double {
    val xx = x.toDouble()
    return xx * xx * xx
}

data class Wheels(val lf : Double, val rf : Double, val lr : Double, val rr : Double)
fun mecanumDrive(direction : Double, velocity : Double, rot : Double) : Wheels {
    val s = Math.sin(direction + Math.PI / 4.0)
    val c = Math.cos(direction + Math.PI / 4.0)
    val m = Math.max(Math.abs(s), Math.abs(c))

    val v1 = velocity * s / m + rot
    val v2 = velocity * c / m - rot
    val v3 = velocity * c / m + rot
    val v4 = velocity * s / m - rot
    val mx = maximum(v1, v2, v3, v4)

    return Wheels(v1 / mx, v2 / mx, v3 / mx, v4 / mx)
}

@TeleOp(name = "Mecanum")
class MecanumTeleop : OpMode() {
    var lf : DcMotor? = null
    var lr : DcMotor? = null
    var rf : DcMotor? = null
    var rr : DcMotor? = null

    override fun init() {
        lf = hardwareMap.dcMotor.get("lf")
        lr = hardwareMap.dcMotor.get("lr")
        rf = hardwareMap.dcMotor.get("rf")
        rr = hardwareMap.dcMotor.get("rr")
    }

    override fun loop() {
        val lx = cube(gamepad1.left_stick_x)
        val ly = cube(- gamepad1.left_stick_y)
        val rot = cube(gamepad1.right_stick_x)

        val theta = Math.atan2(lx, ly)
        val mag = Math.sqrt(lx * lx + ly * ly)

        drive(theta, mag, rot)
    }

    private fun drive(direction : Double, velocity: Double, rot : Double) {
        val (lfp, rfp, lrp, rrp) = mecanumDrive(direction, velocity, rot)

        lf?.power = lfp
        rf?.power = rfp
        lr?.power = lrp
        rr?.power = rrp
    }
}