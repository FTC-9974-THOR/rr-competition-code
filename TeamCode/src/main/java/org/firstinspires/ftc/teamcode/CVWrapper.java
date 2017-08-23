package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;
import android.media.Image;

/**
 * Created by FTC on 11/5/2016.
 */
@SuppressWarnings("ALL")
public interface CVWrapper {
    boolean init();

    boolean capture();

    Image getRaw();

    Camera.ShutterCallback getCallback();

    void setCallback(Camera.ShutterCallback call);
}
