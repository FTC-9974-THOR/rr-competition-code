package org.firstinspires.ftc.teamcode.robotModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.KiwiDrive;

/**
 * Created by FTC on 6/9/2017.
 */

@TeleOp(name="KI3", group="Robots")
public class KI3 extends OpMode {

    KiwiDrive kiwiDrive;

    Gamepad prevGamepadState;

    @Override
    public void init() {
        kiwiDrive = new KiwiDrive(hardwareMap, this);
        kiwiDrive.integrate = true;
        prevGamepadState = gamepad1;
    }

    @Override
    public void loop() {
        if (!kiwiDrive.ready) {
            if (time % 2.0 <= 1.0) {
                telemetry.addLine("CALIBRATION REQUIRED");
            } else {
                telemetry.addLine();
            }
            telemetry.addLine("Calibration status");
            byte calStatus = kiwiDrive.imu.getCalibrationStatus().calibrationStatus;
            telemetry.addData("SYS Cal", (calStatus >> 6) & 0x3);
            telemetry.addData("GYR Cal", (calStatus >> 4) & 0x3);
            telemetry.addData("ACC Cal", (calStatus >> 2) & 0x3);
            telemetry.addData("MAG Cal", calStatus & 0x3);
        } else {
            if (!prevGamepadState.a && gamepad1.a) {
                kiwiDrive.resetIntegrator();
            }
            kiwiDrive.move(-gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
            telemetry.addData("X", kiwiDrive.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("Y", kiwiDrive.imu.getAngularOrientation().secondAngle);
            telemetry.addData("Z", kiwiDrive.imu.getAngularOrientation().firstAngle);
        }

        prevGamepadState = gamepad1;
    }
}
