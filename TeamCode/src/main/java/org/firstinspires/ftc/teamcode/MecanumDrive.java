package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * Created by FTC on 4/30/2017.
 */
public class MecanumDrive {
    DcMotor lf, lb, rf, rb;
    double movementMag, movementTheta;
    double largestVector;
    double lfVector, rfVector, lbVector, rbVector;
    double lfNorm, rfNorm, lbNorm, rbNorm;

    public MecanumDrive(HardwareMap hw) {
        lf = hw.dcMotor.get("LF");
        rf = hw.dcMotor.get("RF");
        lb = hw.dcMotor.get("LB");
        rb = hw.dcMotor.get("RB");

        // TODO: Add encoders :)

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
    }

    public void move(double x, double y, double rot) {
        movementMag = Range.clip(Math.sqrt((x * x) + (y * y)), -1, 1);
        // Java thinks forward is to the right, at 0 radians.
        if (movementMag > 0.1) {
            movementTheta = Math.toDegrees(Math.atan2(x, y) + Math.PI);
        } else {
            movementTheta = 0;
        }

        lfVector = movementMag * Math.sin(Math.toRadians(movementTheta) + (Math.PI / 4)) + rot;
        rfVector = movementMag * Math.cos(Math.toRadians(movementTheta) + (Math.PI / 4)) - rot;
        lbVector = movementMag * Math.cos(Math.toRadians(movementTheta) + (Math.PI / 4)) + rot;
        rbVector = movementMag * Math.sin(Math.toRadians(movementTheta) + (Math.PI / 4)) - rot;

        double[] vectors = new double[]{Math.abs(lfVector), Math.abs(rfVector), Math.abs(lbVector), Math.abs(rbVector)};
        Arrays.sort(vectors);
        largestVector = vectors[vectors.length - 1];

        if (largestVector == 0) {
            largestVector = 1;
        }

        lfNorm = lfVector / largestVector;
        rfNorm = rfVector / largestVector;
        lbNorm = lbVector / largestVector;
        rbNorm = rbVector / largestVector;

        lf.setPower(lfNorm);
        rf.setPower(rfNorm);
        lb.setPower(lbNorm);
        rb.setPower(rbNorm);
    }
}
