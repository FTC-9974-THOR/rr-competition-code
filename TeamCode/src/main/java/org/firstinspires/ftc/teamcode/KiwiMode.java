package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

/**
 * Created by FTC on 6/6/2017.
 */
@TeleOp(name="Kiwi")
public class KiwiMode extends OpMode {

    KiwiDrive kd;

    @Override
    public void init() {
        kd = new KiwiDrive(hardwareMap, this);
    }

    @Override
    public void loop() {
        if (kd.ready) {
            telemetry.addData("X", kd.imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).firstAngle);
            telemetry.addData("Y", kd.imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).secondAngle);
            telemetry.addData("Z", kd.imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle);
        }
        kd.debug = gamepad1.left_bumper;
        kd.integrate = gamepad1.right_bumper;
        kd.move(-gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
    }
}
