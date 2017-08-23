package org.firstinspires.ftc.teamcode;

//import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 11/13/2015.
 */
public class OvertrollMode2 extends LinearOpMode {
    DcMotor LeftFrontMotor;
    DcMotor LeftBackMotor;
    DcMotor RightFrontMotor;
    DcMotor RightBackMotor;
    double pos = 0.01;
    double pos2 = 0.99;
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    Servo Servo4;

    @Override
    public void runOpMode() throws InterruptedException {


        LeftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        LeftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
        RightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        RightBackMotor= hardwareMap.dcMotor.get("right_back_drive");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        LeftFrontMotor.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        LeftBackMotor.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        RightFrontMotor.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        RightBackMotor.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );


        Servo1 = hardwareMap.servo.get("servo1");
        Servo2 = hardwareMap.servo.get("servo2");

        Servo1.setPosition(pos);
        Servo2.setPosition(pos2);
        Servo3 = hardwareMap.servo.get("servo3");
        Servo4 = hardwareMap.servo.get("servo4");

        double pos3 = 0.8; //Servo3.getPosition();
        Servo3.setPosition(pos3);

        double pos4 = Servo4.getPosition();
        Servo4.setPosition(pos4);

        waitForStart();


         //At start...
        pos += 1;

        pos = Range.clip(pos, 0.01, 0.99);
        Servo1.setPosition(pos);

        sleep(100);

        pos -= 1;

        pos = Range.clip(pos, 0.01, 0.99);
        Servo1.setPosition(pos);


        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitOneFullHardwareCycle();

        int count = 15200;
        int avg = (Math.abs(LeftFrontMotor.getCurrentPosition()) + Math.abs(LeftBackMotor.getCurrentPosition())
                + Math.abs(RightFrontMotor.getCurrentPosition()) + Math.abs(RightBackMotor.getCurrentPosition())) / 4;
        LeftFrontMotor.setPower(1);
        LeftBackMotor.setPower(1);
        RightFrontMotor.setPower(1);
        RightBackMotor.setPower(1);
        while (avg < count) {
            telemetry.addData("LF: ", LeftFrontMotor.getCurrentPosition());
            telemetry.addData("LB: ", LeftBackMotor.getCurrentPosition());
            telemetry.addData("RF: ", RightFrontMotor.getCurrentPosition());
            telemetry.addData("RB: ", RightBackMotor.getCurrentPosition());
            avg = (Math.abs(LeftFrontMotor.getCurrentPosition()) + Math.abs(LeftBackMotor.getCurrentPosition())
                    + Math.abs(RightFrontMotor.getCurrentPosition()) + Math.abs(RightBackMotor.getCurrentPosition())) / 4;
            telemetry.addData("AVG: ", avg);
            telemetry.addData("LF power = ", LeftFrontMotor.getPower());
            telemetry.addData("RF power = ", RightFrontMotor.getPower());
            telemetry.addData("LB power = ", LeftBackMotor.getPower());
            telemetry.addData("RB power = ", RightBackMotor.getPower());
            telemetry.addData("LF target = ", LeftFrontMotor.getTargetPosition());
            telemetry.addData("RF target = ", RightFrontMotor.getTargetPosition());
            telemetry.addData("LB target = ", LeftBackMotor.getTargetPosition());
            telemetry.addData("RB target = ", RightBackMotor.getTargetPosition());

        }

        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        RightBackMotor.setPower(0);
    }
}
