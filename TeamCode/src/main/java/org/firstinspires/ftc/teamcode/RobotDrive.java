package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC on 10/19/2016.
 *
 * An instantiatable object that is used for easy configuration of
 * the robot.
 */
@SuppressWarnings("unused")
public class RobotDrive {
    static public final int FOUR_WHEEL = 0;     // 4 wheel drive
    static public final int TWO_WHEEL = 1;      // Ranger bot

    public enum MODES {
        RANGER,
        TANK,
        KIWI,
        HOLONOMIC,
    }

    private int mode;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    DcMotor[] motors;

    private double curveSensitivity = 0.5;

    /**
     * Construct a new robot config
     * @param mode RANGER, TANK, KIWI, or HOLONOMIC
     * @param hw hardware map
     */
    public RobotDrive(MODES mode, HardwareMap hw) {
        switch (mode) {
            case RANGER:
                frontLeft = hw.dcMotor.get("FL");
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight = hw.dcMotor.get("FR");
                // TODO find a better way to do this
                motors = new DcMotor[] {frontLeft, frontRight};
                // Don't forget this, or else force a null check in line 111
                this.mode = TWO_WHEEL;
                break;
        }
    }

    /**
     * Returns a new RobotDrive object. Object oriented programming for the win!
     * @param configNumber determines what robot config to use
     * @param hardwareMap pass OpMode's hardwareMap so this class can use it
     */
    // TODO add custom configs
    public RobotDrive(int configNumber, HardwareMap hardwareMap) throws IllegalArgumentException {
        switch (configNumber) {
            case FOUR_WHEEL:
                mode = 0;
                frontLeft = hardwareMap.dcMotor.get("FL");
                frontRight = hardwareMap.dcMotor.get("FR");
                backLeft = hardwareMap.dcMotor.get("BL");
                backRight = hardwareMap.dcMotor.get("BR");
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case TWO_WHEEL:
                mode = 1;
                frontLeft = hardwareMap.dcMotor.get("FL");
                frontRight = hardwareMap.dcMotor.get("FR");
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                // Because we don't use them
                // TODO find something better, like maybe some type of destroy()
                backLeft = null;
                backRight = null;
                break;
            default:
                throw new IllegalArgumentException("Error! Use a valid config");
        }
    }

    /**
     * Get which mode the object is in
     * @return mode
     */
    public int getMode() {
        return mode;
    }

    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    public void update (double left, double right) {
        this.tankUpdate(left, right);
    }

    /**
     * Built in tank drive
     * @param left left input
     * @param right right input
     */
    public void tankUpdate(double left, double right) {
        switch (mode) {
            case FOUR_WHEEL:
                frontLeft.setPower(left);
                backLeft.setPower(left);
                frontRight.setPower(right);
                backRight.setPower(right);
                break;
            case TWO_WHEEL:
                frontLeft.setPower(left);
                frontRight.setPower(right);
                break;
            default:
                break;
        }
    }
/**
     * Set the encoder targets for the motors
     * @param leftTarget new encoder target
     * @param rightTarget new encoder target
     */
    public void setMotorTargets(int leftTarget, int rightTarget) {
        this.frontLeft.setTargetPosition(leftTarget);
        this.frontRight.setTargetPosition(rightTarget);
    }

    /**
     * Returns true if the motors are busy.
     * @param both if both motors must be busy to return true
     * @return if the motors are busy
     */
    public boolean areMotorsBusy(boolean both) {
        if (both) {
            return (frontRight.isBusy() && frontLeft.isBusy());
        } else {
            return (frontRight.isBusy() || frontLeft.isBusy());
        }
    }

    /**
     * The RobotDrive.Drive command, ported from the C++ WPIlib
     * @param power power of motors
     * @param curve angle to turn
     */
    public void curve(double power, double curve) {
        double leftOut;
        double rightOut;

        if (curve < 0) {
            double curveLog = Math.log(-curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power / ratio;
            rightOut = power;
        } else if (curve > 0) {
            double curveLog = Math.log(curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power;
            rightOut = power / ratio;
        } else {
            leftOut = power;
            rightOut = power;
        }

        this.update(leftOut, rightOut);
    }

    /**
     * Like curve(), but it does it in place
     * @param power power of motors
     * @param curve angle to turn
     */
    public void countersteerCurve(double power, double curve) {
        double leftOut;
        double rightOut;

        if (curve < 0) {
            double curveLog = Math.log(-curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = -power / ratio;
            rightOut = power / ratio;
        } else if (curve > 0) {
            double curveLog = Math.log(curve);
            double ratio = (curveLog - curveSensitivity) / (curveLog + curveSensitivity);
            ratio = (ratio == 0) ? 0.0000000001 : ratio;
            leftOut = power / ratio;
            rightOut = -power / ratio;
        } else {
            leftOut = power;
            rightOut = power;
        }

        this.update(leftOut, rightOut);
    }
}
