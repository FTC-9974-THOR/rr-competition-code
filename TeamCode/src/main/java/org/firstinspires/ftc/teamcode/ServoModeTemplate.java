package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/8/2015.
 */

public class ServoModeTemplate extends OpMode {

    DcMotor LeftFrontMotor;
    DcMotor LeftBackMotor;
    DcMotor RightFrontMotor;
    DcMotor RightBackMotor;

    @Override
    public void init(){
        //get references to the motors from the hardware map
        LeftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        LeftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        RightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        RightBackMotor= hardwareMap.dcMotor.get("right_back_drive");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
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

        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
       /* telemetry.addData("LF: ", LeftFrontMotor.getCurrentPosition());
        telemetry.addData("LB: ", LeftBackMotor.getCurrentPosition());
        telemetry.addData("RF: ", RightFrontMotor.getCurrentPosition());
        telemetry.addData("RB: ", RightBackMotor.getCurrentPosition());
        telemetry.addData("LF power = ", LeftFrontMotor.getPower());
        telemetry.addData("RF power = ", RightFrontMotor.getPower());
        telemetry.addData("LB power = ", LeftBackMotor.getPower());
        telemetry.addData("RB power = ", RightBackMotor.getPower());
        telemetry.addData("LF target = ", LeftFrontMotor.getTargetPosition());
        telemetry.addData("RF target = ", RightFrontMotor.getTargetPosition());
        telemetry.addData("LB target = ", LeftBackMotor.getTargetPosition());
        telemetry.addData("RB target = ", RightBackMotor.getTargetPosition());*/
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
