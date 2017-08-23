package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 8/16/2016.
 */
public class RangerBotTemplate extends OpMode {
    DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("l");
        right = hardwareMap.dcMotor.get("r");
        left.setDirection(DcMotor.Direction.REVERSE);
    }

    double l, r = 0;
    @Override
    public void loop() {
        l = gamepad1.left_stick_y;
        r = gamepad1.right_stick_y;

        l = Range.clip(l, -1, 1);
        r = Range.clip(r, -1, 1);

        left.setPower(l);
        right.setPower(r);
    }
}
