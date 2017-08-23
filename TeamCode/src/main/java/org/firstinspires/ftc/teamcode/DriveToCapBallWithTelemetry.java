package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by FTC on 10/15/2015.
 */
@Autonomous(name="Drive to cap ball telemetry", group="2017")
public class DriveToCapBallWithTelemetry extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;

    final int motorTime = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftMotor = hardwareMap.dcMotor.get("FL");
        RightMotor = hardwareMap.dcMotor.get("FR");
        RightMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        for(int i=0; i<1; i++) {
            LeftMotor.setPower(1.0);
            RightMotor.setPower(1.0);
            for(int s = 0; s < motorTime; i += 2) {
                telemetry.addData("Right encoder", RightMotor.getCurrentPosition());
                telemetry.addData("Left encoder", LeftMotor.getCurrentPosition());
                updateTelemetry(telemetry);
                sleep(2);
            }
        }

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);
        //telemetry.addData("Encoder - raw - Left", LeftMotor.getCurrentPosition());
        //telemetry.addData("Encoder - raw - Right", RightMotor.getCurrentPosition());



    }


}