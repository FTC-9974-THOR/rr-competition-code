package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by FTC on 10/19/2016.
 *
 * This is the first test of the 2017 code.
 */

//TODO add suppress spelling annotation
@TeleOp(name="This is a test", group="2017")
//@Disabled
public class VelocityVortexTest extends OpMode {

    RobotDrive robotDrive;

    /**
     * Run once, when the robot is initialized.
     */
    @Override
    public void init() {
        robotDrive = new RobotDrive(RobotDrive.TWO_WHEEL, hardwareMap);
    }

    /**
     * Run over and over when the robot is enabled
     */
    @Override
    public void loop() {
        robotDrive.tankUpdate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }
}
