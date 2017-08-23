package org.firstinspires.ftc.teamcode.robotModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Created by FTC on 4/23/2017.
 */
@TeleOp(name="Newt", group="Robots")
public class Newt extends OpMode {

    MecanumDrive md;

//    DcMotor leftFrontMotor; /**1*/
//    DcMotor leftBackMotor; /**3*/
//    DcMotor rightFrontMotor; /**2*/
//    DcMotor rightBackMotor;  /**4*/

    @Override
    public void init() {

        md = new MecanumDrive(hardwareMap);
//        leftFrontMotor = hardwareMap.dcMotor.get("LF");
//        leftBackMotor = hardwareMap.dcMotor.get("LB");
//        rightFrontMotor = hardwareMap.dcMotor.get("RF");
//        rightBackMotor = hardwareMap.dcMotor.get("RB");
//
//        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        //Math.cos();
    }

    @Override
    public void loop() {
        md.move(gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
        /*telemetry.addData("Mag", md.movementMag);
        telemetry.addData("Theta", md.movementTheta);
        telemetry.addData("LF", md.lfVector);
        telemetry.addData("RF", md.rfVector);
        telemetry.addData("LB", md.lbVector);
        telemetry.addData("RB", md.rbVector);
        telemetry.addData("LFN", md.lfNorm);
        telemetry.addData("RFN", md.rfNorm);
        telemetry.addData("LBN", md.lbNorm);
        telemetry.addData("RBN", md.rbNorm);*/
//        leftFrontMotor.setPower(-gamepad1.left_stick_y);
    }
}
