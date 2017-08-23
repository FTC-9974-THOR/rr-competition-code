package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC on 1/15/2017.
 */
@TeleOp(name="Basic Driving", group="2017")
public class BasicDrive extends OpMode {
    RobotDrive robot;

    @Override
    public void init() {
        robot = new RobotDrive(RobotDrive.TWO_WHEEL, hardwareMap);
    }

    @Override
    public void loop() {
        robot.tankUpdate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }
}
