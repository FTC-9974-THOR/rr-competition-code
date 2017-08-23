package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by FTC on 11/12/2016.
 */
public class GyroTest extends OpMode implements SensorListener {
    Sensor gyro;
    SensorManager manager;
    @Override
    @SuppressWarnings("deprecation")
    public void init() {
        gyro = manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        manager.registerListener(this, Sensor.TYPE_GYROSCOPE);
    }

    @Override
    public void loop() {

    }

    @Override
    public void onSensorChanged(int sensor, float[] values) {
        if (sensor == gyro.getType()) {
            telemetry.addData("X", values[0]);
            telemetry.addData("Y", values[1]);
            telemetry.addData("Z", values[2]);
            telemetry.addData("Raw", values);
        }
    }

    @Override
    public void onAccuracyChanged(int sensor, int accuracy) {

    }
}
