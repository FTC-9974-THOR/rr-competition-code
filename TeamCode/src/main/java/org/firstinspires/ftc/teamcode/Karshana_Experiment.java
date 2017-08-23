package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 11/6/2016.
 */
@TeleOp(name="Experimental TeleOp",group = "2017")
public class Karshana_Experiment extends OpMode{

    RobotDrive rb;

    DcMotor intake;

    @Override
    public void init() {

        rb = new RobotDrive(RobotDrive.TWO_WHEEL,hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");

    }

    @Override
    public void loop() {
        rb.tankUpdate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        if (gamepad2.b) {
            intake.setPower(1);
        } else if (gamepad2.a) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        telemetry.addLine("Encoder values");
        telemetry.addData("Left Drive", rb.frontLeft.getCurrentPosition());
        telemetry.addData("Right Drive", rb.frontRight.getCurrentPosition());
        telemetry.addData("Intake", intake.getCurrentPosition());
    }
}
