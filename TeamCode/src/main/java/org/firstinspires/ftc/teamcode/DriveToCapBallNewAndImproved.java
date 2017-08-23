package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by FTC on 11/23/2016.
 */
@Autonomous(name="Drive to cap ball with encoders", group="2017")
public class DriveToCapBallNewAndImproved extends LinearOpMode {
    DcMotor LeftMotor;
    DcMotor RightMotor;
    DcMotor leftLauncher;
    DcMotor rightLauncher;
    DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 420 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;



    // How long (in milliseconds) to run the motors, do not change
    final int driveTime = 2500;

    @Override
    public void runOpMode() throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

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

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        leftLauncher.setPower(0.05);
        rightLauncher.setPower(-0.05);

        sleep(3000);

        intake.setPower(0.7);

        sleep(1500);

        intake.setPower(0);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);

        sleep(5500);

        int driveDistanceInInches = 60;

        double timeoutS = 6;

        // Determine new target position, and pass to motor controller
        newLeftTarget = LeftMotor.getCurrentPosition() - (int)(driveDistanceInInches * COUNTS_PER_INCH);
        newRightTarget = RightMotor.getCurrentPosition() - (int)(driveDistanceInInches * COUNTS_PER_INCH);
        LeftMotor.setTargetPosition(newLeftTarget);
        RightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        LeftMotor.setPower(-1);
        RightMotor.setPower(-1);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (LeftMotor.isBusy() && RightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    LeftMotor.getCurrentPosition(),
                    RightMotor.getCurrentPosition());
            telemetry.update();

            // Allow time for other processes to run.
            idle();
        }

        /*LeftMotor.setPower(-1.0);
        RightMotor.setPower(-1.0);

        sleep(driveTime);
        */

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);

        sleep(100);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(-1.0);

        sleep(300);

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);



    }


}