package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/8/2015.
 */

public class ServoMode2 extends OpMode {
    double servo1Position = 0.01;
    double servo2Position = 0.99;
    double servo3Position = 0.75;
    double servo4Position = 0.01;

    DcMotor LeftFrontMotor;
    DcMotor LeftBackMotor;
    DcMotor RightFrontMotor;
    DcMotor RightBackMotor;
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    Servo Servo4;
    double avg = 0;


    @Override
    public void init(){
        //get references to the motors from the hardware map
        LeftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        LeftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        RightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        RightBackMotor= hardwareMap.dcMotor.get("right_back_drive");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        Servo1 = hardwareMap.servo.get("servo1");
        Servo2 = hardwareMap.servo.get("servo2");
        Servo3 = hardwareMap.servo.get("servo3");
        Servo4 = hardwareMap.servo.get("servo4");

        telemetry.addData("servo4Position = ", Servo4.getPosition());

        //Servo4.setPosition(servo4Position);

    }
    @Override
    public void loop() {


        double right = -gamepad1.left_stick_y;
        double left = -gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = scaleInput(right);
        left =  scaleInput(left);

        RightFrontMotor.setPower(right);
        RightBackMotor.setPower(right);
        LeftFrontMotor.setPower(left);
        LeftBackMotor.setPower(left);


        if (gamepad2.x) {
            servo1Position += 0.05;
        }
        if (gamepad2.a) {
            servo1Position -= 0.05;
        }
        servo1Position = Range.clip(servo1Position, 0.01, 0.99);

        if (gamepad2.y) {
            servo2Position -= 0.05;
        }
        if (gamepad2.b) {
            servo2Position += 0.05;
        }
        servo2Position = Range.clip(servo2Position, 0.01, 0.99);

        if (gamepad2.right_bumper) {
            servo3Position -= 0.05;
        }
        if (gamepad2.right_trigger >= 0.9F) {
            servo3Position += 0.05;
        }
        servo3Position = Range.clip(servo3Position, 0.20, 0.80);

        if (gamepad2.left_bumper) {
            servo4Position -= 0.05;
        }
        if (gamepad2.left_trigger >= 0.9F) {
            servo4Position += 0.05;
        }
        servo4Position = Range.clip(servo4Position, 0.01, 0.99);

        // Set servo states
        Servo1.setPosition(servo1Position);
        Servo2.setPosition(servo2Position);
        Servo3.setPosition(servo3Position);
        Servo4.setPosition(servo4Position);

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
        telemetry.addData("servo1Position = ", servo1Position);
        telemetry.addData("servo2Position = ", servo2Position);
        telemetry.addData("servo3Position = ", servo3Position);
        telemetry.addData("servo4Position = ", servo4Position);

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
