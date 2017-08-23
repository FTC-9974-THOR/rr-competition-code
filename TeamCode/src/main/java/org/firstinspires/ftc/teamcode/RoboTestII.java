package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/8/2015.
 */
public class RoboTestII extends OpMode {

    DcMotor Front;
    DcMotor Back;
    DcMotor Left;
    DcMotor Right;

    OpticalDistanceSensor optical;
    @Override
    public void init(){
        //get references to the motors from the hardware map
        Front = hardwareMap.dcMotor.get("front");
        Back = hardwareMap.dcMotor.get("back");
        Left = hardwareMap.dcMotor.get("left");
        Right= hardwareMap.dcMotor.get("right");
        Back.setDirection(DcMotor.Direction.REVERSE);
        Left.setDirection(DcMotor.Direction.REVERSE);

        optical = hardwareMap.opticalDistanceSensor.get("sensor");


    }
    @Override
    public void loop() {
        double rotate = gamepad1.left_stick_x;

        double translateFB = gamepad1.right_stick_y;
        double translateSS = -gamepad1.right_stick_x;

        double speedF, speedB, speedL, speedR = 0;

        translateSS = Range.clip(translateSS, -1, 1);
        translateFB = Range.clip(translateFB, -1, 1);

        rotate = Range.scale(rotate, -1, 1, -2, 2);

        speedF = Range.clip((translateSS + rotate), -0.6, 0.6);
        speedB = Range.clip((translateSS - rotate), -0.6, 0.6);
        speedL = Range.clip((translateFB - rotate), -0.6, 0.6);
        speedR = Range.clip((translateFB + rotate), -0.6, 0.6);
        if (speedF != Range.clip((translateSS + rotate), -0.6, 0.6)) {
            if (speedF < Range.clip((translateSS + rotate), -0.6, 0.6)) {
                speedF += 0.1;
            } else {
                speedF -= 0.1;
            }
        }
        if (speedB != Range.clip((translateSS - rotate), -0.6, 0.6)) {
            if (speedB < Range.clip((translateSS - rotate), -0.6, 0.6)) {
                speedB += 0.1;
            } else {
                speedB -= 0.1;
            }
        }
        if (speedL != Range.clip((translateFB - rotate), -0.6, 0.6)) {
            if (speedL < Range.clip((translateFB - rotate), -0.6, 0.6)) {
                speedL += 0.1;
            } else {
                speedL -= 0.1;
            }
        }
        if (speedR != Range.clip((translateFB + rotate), -0.6, 0.6)) {
            if (speedR < Range.clip((translateFB + rotate), -0.6, 0.6)) {
                speedR += 0.1;
            } else {
                speedR -= 0.1;
            }
        }
        Left.setPower(speedL);
        Right.setPower(speedR);
        Front.setPower(speedF);
        Back.setPower(speedB);
        boolean turnonlight = gamepad1.left_bumper;

        optical.enableLed(turnonlight);
        telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", translateFB));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", translateSS));
    }
}
