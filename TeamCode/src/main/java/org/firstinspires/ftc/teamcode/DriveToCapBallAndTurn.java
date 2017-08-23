package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by FTC on 10/15/2015.
 */
@Autonomous(name="Drive to cap ball and turn", group="2017")
public class DriveToCapBallAndTurn extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;

    // How long (in milliseconds) to run the motors
    final int motorTime = 2500;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftMotor = hardwareMap.dcMotor.get("FL");
        RightMotor = hardwareMap.dcMotor.get("FR");
        RightMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("");

        waitForStart();

        sleep(10000);

        for(int i=0; i<1; i++) {
            LeftMotor.setPower(-1.0);
            RightMotor.setPower(-1.0);

            sleep(motorTime);
        }

        LeftMotor.setPower(-0.8);
        RightMotor.setPower(-0.2);

        sleep(2000);

        LeftMotor.setPower(0);
        RightMotor.setPower(0);



    }


}