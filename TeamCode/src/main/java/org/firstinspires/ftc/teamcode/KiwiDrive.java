package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.Arrays;

/**
 * Created by FTC on 6/6/2017.
 */
public class KiwiDrive {

    DcMotor wheel1, wheel2, wheel3;
    double w1Vector, w2Vector, w3Vector, w1Norm, w2Norm, w3Norm;

    public BNO055IMU imu;
    public boolean debug = false;
    public boolean integrate = false;
    public boolean ready = false;

    private boolean run = false;
    private double originalZ;

    Runnable calibration = new Runnable() {
        @Override
        public void run() {
            while (!(imu.isSystemCalibrated() || imu.isMagnetometerCalibrated())) {
                root.telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
                root.telemetry.update();
                Thread.yield();
            }
            ready = true;
        }
    };

    OpMode root;

    public KiwiDrive(HardwareMap hw, OpMode root) {
        this.root = root;

        wheel1 = hw.dcMotor.get("W1");
        wheel2 = hw.dcMotor.get("W2");
        wheel3 = hw.dcMotor.get("W3");

        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.mode = BNO055IMU.SensorMode.NDOF;
        if (imu.initialize(p)) {
            root.telemetry.addLine("IMU initialisation successful");
            root.telemetry.update();
        } else {
            root.telemetry.addLine("IMU initialisation failed!");
            root.telemetry.addData("Reason (SYS_ERR)", String.format("%x", imu.getSystemError().bVal));
            root.telemetry.update();
            RobotLog.setGlobalErrorMsg("IMU initialisation failed!");
            root.requestOpModeStop();
            return;
        }

        Thread calibrationThread = new Thread(calibration);
        calibrationThread.start();
    }

    public void move(double x, double y, double rot) {
        double _x = -x;
        double _rot = -rot;

        if (!ready) {
            if (root.time % 2.0 <= 1.0) {
                root.telemetry.addLine("CALIBRATION NEEDED");
            } else {
                root.telemetry.addLine();
            }
            return;
        }

        if (!run) {
            originalZ = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
            run = true;
        }

        double integratedZ = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle - originalZ;
        root.telemetry.addData("Int Z", integratedZ);

        w1Vector = (Math.cos(((2.0 * Math.PI) / 3.0) + ((integrate) ? integratedZ : 0)) * _x) + (Math.sin(((2.0 * Math.PI) / 3.0) + ((integrate) ? integratedZ : 0)) * y) + _rot;
        w2Vector = (Math.cos(((4.0 * Math.PI) / 3.0) + ((integrate) ? integratedZ : 0)) * _x) + (Math.sin(((4.0 * Math.PI) / 3.0) + ((integrate) ? integratedZ : 0)) * y) + _rot;
        w3Vector = (Math.cos((integrate) ? integratedZ : 0) * _x) + (Math.sin((integrate) ? integratedZ : 0) * y) + _rot;

        if (debug) {
            double[] vectors = new double[]{Math.abs(w1Vector), Math.abs(w2Vector), Math.abs(w3Vector)};
            Arrays.sort(vectors);

            w1Norm = Range.scale(w1Vector, -vectors[vectors.length - 1], vectors[vectors.length - 1], -1, 1);
            RobotLog.d(Boolean.toString(w1Norm == ((vectors[vectors.length - 1] == 0) ? 0 : (w1Vector / vectors[vectors.length - 1]))));
            w2Norm = Range.scale(w2Vector, -vectors[vectors.length - 1], vectors[vectors.length - 1], -1, 1);
            w3Norm = Range.scale(w3Vector, -vectors[vectors.length - 1], vectors[vectors.length - 1], -1, 1);

            wheel1.setPower(w1Norm);
            wheel2.setPower(w2Norm);
            wheel3.setPower(w3Norm);
        } else {
            wheel1.setPower(w1Vector);
            wheel2.setPower(w2Vector);
            wheel3.setPower(w3Vector);
        }
        root.telemetry.addData("W1", wheel1.getPower());
        root.telemetry.addData("W2", wheel2.getPower());
        root.telemetry.addData("W3", wheel3.getPower());
    }

    public void resetIntegrator() {
        originalZ = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
    }
}
