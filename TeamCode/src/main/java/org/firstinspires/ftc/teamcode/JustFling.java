package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Created by FTC on 10/15/2015.
 */
@Autonomous(name="Just Fling", group="2017")
public class JustFling extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;
    DcMotor leftLauncher;
    DcMotor rightLauncher;
    DcMotor intake;



    // How long (in milliseconds) to run the motors
    final int driveTime = 2500;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftMotor = hardwareMap.dcMotor.get("FL");
        RightMotor = hardwareMap.dcMotor.get("FR");
        RightMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher = hardwareMap.dcMotor.get("LL");
        rightLauncher = hardwareMap.dcMotor.get("RL");
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.dcMotor.get("intake");

        telemetry.addLine("");

        waitForStart();


        leftLauncher.setPower(0.6);
        rightLauncher.setPower(-0.6);

        sleep(3000);

        intake.setPower(0.7);

        sleep(1500);

        intake.setPower(0);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);

        sleep(5500);



    }


}