package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Method;

/**
 * Created by FTC on 12/3/2016.
 */
public class Utility {
    static final double     COUNTS_PER_MOTOR_REV    = 420 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private boolean stopRequested = false;
    private Class callerClass;
    private Method callerStop;
    private Utility(Class caller) {
        callerClass = caller.getSuperclass();
        //callerStop = callerClass.getMethod("stopRequested")
    }

    private void TeleOpSleep(long millis) {
        for (long i = 0; i < millis; i++) {
            //if
        }
    }
}
