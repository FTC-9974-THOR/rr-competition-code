package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/8/2015.
 */

public class ServoMode3 extends OpMode {
    double pos = 0.01;
    double pos2 = 0.99;

    DcMotor LeftFrontMotor;
    DcMotor LeftBackMotor;
    DcMotor RightFrontMotor;
    DcMotor RightBackMotor;
    DcMotor SlideMotor;
    Servo Servo1;
    Servo Servo2;

    @Override
    public void init(){
        //get references to the motors from the hardware map
        LeftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        LeftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        RightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        RightBackMotor= hardwareMap.dcMotor.get("right_back_drive");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        SlideMotor = hardwareMap.dcMotor.get("slide_motor");

        Servo1 = hardwareMap.servo.get("servo1");
        Servo2 = hardwareMap.servo.get("servo2");



    }
    @Override
    public void loop() {


        float right = -gamepad1.left_stick_y;
        float left = -gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        RightFrontMotor.setPower(right);
        RightBackMotor.setPower(right);
        LeftFrontMotor.setPower(left);
        LeftBackMotor.setPower(left);


        if (gamepad2.x) {
            pos += 0.05;
        }
        if (gamepad2.a) {
            pos -= 0.05;
        }
        pos = Range.clip(pos, 0.01, 0.99);
        Servo1.setPosition(pos);


        if (gamepad2.y) {
            pos2 -= 0.05;
        }
        if (gamepad2.b) {
            pos2 += 0.05;
        }
        pos2 = Range.clip(pos2, 0.01, 0.99);
        Servo2.setPosition(pos2);

        if (gamepad1.dpad_up) {
            SlideMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            SlideMotor.setPower(-0.5);
        } else {
            SlideMotor.setPower(0.0);
        }




        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("LF: ", LeftFrontMotor.getCurrentPosition());
        telemetry.addData("LB: ", LeftBackMotor.getCurrentPosition());
        telemetry.addData("RF: ", RightFrontMotor.getCurrentPosition());
        telemetry.addData("RB: ", RightBackMotor.getCurrentPosition());
        /*telemetry.addData("AVG: ", (LeftFrontMotor.getCurrentPosition() + LeftBackMotor.getCurrentPosition()
                + RightFrontMotor.getCurrentPosition() + RightBackMotor.getCurrentPosition()) / 4);
        avg = (LeftFrontMotor.getCurrentPosition() + LeftBackMotor.getCurrentPosition()
                + RightFrontMotor.getCurrentPosition() + RightBackMotor.getCurrentPosition()) / 4; */
        telemetry.addData("LF power = ", LeftFrontMotor.getPower());
        telemetry.addData("RF power = ", RightFrontMotor.getPower());
        telemetry.addData("LB power = ", LeftBackMotor.getPower());
        telemetry.addData("RB power = ", RightBackMotor.getPower());
        telemetry.addData("LF target = ", LeftFrontMotor.getTargetPosition());
        telemetry.addData("RF target = ", RightFrontMotor.getTargetPosition());
        telemetry.addData("LB target = ", LeftBackMotor.getTargetPosition());
        telemetry.addData("RB target = ", RightBackMotor.getTargetPosition());



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
