package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC on 10/8/2015.
 */
@TeleOp (name="MyOwnOpMode2",group = "2017")
public class MyOwnOpMode2 extends OpMode {

    DcMotor LeftMotor;
    DcMotor RightMotor;
    //Servo Servo1;
    //double avg = 0;

    @Override
    public void init(){
        //get references to the motors from the hardware map
        LeftMotor = hardwareMap.dcMotor.get("FL");
        RightMotor = hardwareMap.dcMotor.get("FR");
        LeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Servo1 = hardwareMap.servo.get("servo1");


    }
    @Override
    public void loop() {

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        double servo1Position = 0.0;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        RightMotor.setPower(right);
        LeftMotor.setPower(left);

        if (gamepad2.a) {
            servo1Position += 0.05;
        }
        if (gamepad2.b) {
            servo1Position -= 0.05;
        }
        servo1Position = Range.clip(servo1Position, 0.01, 0.99);


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("servo1Position" , "servo1Position = " + String.format("%.2f",servo1Position));
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

         //get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

         //index should be positive.
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