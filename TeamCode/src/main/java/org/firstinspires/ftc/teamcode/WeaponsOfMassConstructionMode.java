package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 10/23/2016.
 *
 * This is template code for the skeleton of an opmode.
 */
// Uncomment this to make a TeleOp
//@TeleOp(name=/* name goes here */"Hello, World!", group=/* don't mess with this*/"2017")
// Uncomment this to make an Auto
@Autonomous(name=/* name goes here */"Hello, Auto!", group=/* don't mess with this */"2017")
@Disabled
public class WeaponsOfMassConstructionMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor RightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor LeftMotor = hardwareMap.dcMotor.get("FL");

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        int driveDistanceInInches = 48;

        // Determine new target position, and pass to motor controller
        int newLeftTarget = LeftMotor.getCurrentPosition() - (int) (driveDistanceInInches * Utility.COUNTS_PER_INCH);
        int newRightTarget = RightMotor.getCurrentPosition() - (int)(driveDistanceInInches * Utility.COUNTS_PER_INCH);
        LeftMotor.setTargetPosition(newLeftTarget);
        RightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        LeftMotor.setPower(-1);
        RightMotor.setPower(-1);

        while (!isStopRequested()) {
            telemetry.addLine("Driving");
            updateTelemetry(telemetry);
            idle();
        }

        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
}
