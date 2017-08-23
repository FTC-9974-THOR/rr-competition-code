package org.firstinspires.ftc.teamcode.framework.usb;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by FTC on 8/15/2017.
 */
public class DeviceDisconnectedException extends RuntimeException {

    public String message;
    public Throwable cause;

    public DeviceDisconnectedException() {
        super();
    }

    public DeviceDisconnectedException(String message) {
        super(message);
        this.message = message;
    }

    public DeviceDisconnectedException(String message, Throwable cause) {
        super(message, cause);
        this.message = message;
        this.cause = cause;
    }

    public void log() {
        String log = "DeviceDisconnectedException:";
        if (message != "") {
            log = log + " " + message;
        }
        log = log + " Device unplugged (got unexpected SE0)";
        if (cause != null) {
            log = log + " " + cause.getMessage();
        }
        RobotLog.e(log);
    }
}
