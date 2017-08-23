package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC on 10/23/2016.
 *
 * This is template code for the skeleton of an opmode.
 */
// Uncomment this to make a TeleOp
@TeleOp(name=/* name goes here */"Hello, World!", group=/* don't mess with this*/"2017")
// Uncomment this to make an Auto
//@Autonomous(name=/* name goes here */"Hello, Auto!", group=/* don't mess with this */"2017")
public class PIDTest extends OpMode {

    PID pid;
    RobotDrive rb;

    final double p = 1;
    final double i = 1;
    final double d = 1;
    final double setpoint = 1000;

    @Override
    public void init() {
        pid = new PID(p, i, d, 0.0d, setpoint, false, 0, 1500, 0, 1100);
        try {
            rb = new RobotDrive(RobotDrive.TWO_WHEEL, hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addData("Error", "Illegal RobotDrive parameters");
        }
    }

    @Override
    public void loop() {
        rb.tankUpdate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }
}
