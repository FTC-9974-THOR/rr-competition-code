package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Created by FTC on 10/15/2015.
 */
public class Driveinsquare extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftMotor = hardwareMap.dcMotor.get("left_drive");
        RightMotor = hardwareMap.dcMotor.get("right_drive");
        RightMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        for(int i=0; i<4; i++) {
            LeftMotor.setPower(1.0);
            RightMotor.setPower(1.0);

            sleep(2000);

            LeftMotor.setPower(0.5);
            RightMotor.setPower(-0.5);
//1500ms is 90 degrees
            sleep(1500);
        }

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);



    }


}
