package soupbox

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

val EPSILON = 0.0001

fun cube(x : Float) : Double { return Math.pow(x.toDouble(), 3.0) }

@TeleOp(name = "Basic Mecanum")
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
        lf?.direction = DcMotorSimple.Direction.REVERSE
        lr?.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        val lx = cube(gamepad1.left_stick_x)
        val ly = cube(- gamepad1.left_stick_y)
        val rot = cube(gamepad1.right_stick_x)
        val theta = Math.atan2(lx, ly)
        val mag = Math.min(1.0, Math.sqrt(lx * lx + ly * ly))
        drive(theta, mag, rot)
    }

    private fun drive(direction : Double, velocity: Double, rot : Double) {
        val s = velocity * Math.sin(direction + Math.PI / 4.0)
        val c = velocity * Math.cos(direction + Math.PI / 4.0)
        val lfp = s + rot
        val rfp = c - rot
        val lrp = c + rot
        val rrp = s - rot
        val mx = doubleArrayOf(EPSILON, lfp, rfp, lrp, rrp).reduce {
            acc, d -> Math.max(acc, Math.abs(d))
        }
        lf?.power = lfp / mx
        rf?.power = rfp / mx
        lr?.power = lrp / mx
        rr?.power = rrp / mx
    }
}