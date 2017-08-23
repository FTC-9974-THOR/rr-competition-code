package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/8/2015.
 */
public class RoboTestCam extends OpMode {

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

        translateSS = Range.clip(translateSS, -1, 1);
        translateFB = Range.clip(translateFB, -1, 1);

        rotate = Range.scale(rotate, -1, 1, -2, 2);

        Front.setPower(Range.clip((translateSS + rotate), -1, 1));
        Back.setPower(Range.clip((translateSS - rotate), -1, 1));
        Left.setPower(Range.clip((translateFB - rotate), -1, 1));
        Right.setPower(Range.clip((translateFB + rotate), -1, 1));

        boolean turnonlight = gamepad1.left_bumper;

        optical.enableLed(turnonlight);
        telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", translateFB));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", translateSS));
        telemetry.addData("optical sensor", optical.getLightDetected());


    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    } 



}
