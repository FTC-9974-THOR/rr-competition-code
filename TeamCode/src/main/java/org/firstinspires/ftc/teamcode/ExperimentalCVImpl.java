package org.firstinspires.ftc.teamcode;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.media.Image;
import android.media.ImageReader;

/**
 * Created by FTC on 11/5/2016.
 */
public class ExperimentalCVImpl implements CVWrapper {
    Camera cam;
    Camera.ShutterCallback shutterCallback;
    Camera.PictureCallback raw;
    Camera.PictureCallback jpeg;
    byte camData[];
    Image parsedImage;

    @Override
    public boolean init() {
        cam = Camera.open();

        shutterCallback = new Camera.ShutterCallback() {
            @Override
            public void onShutter() {

            }
        };
        raw = new Camera.PictureCallback() {
            @Override
            public void onPictureTaken(byte[] data, Camera camera) {
                camData = data;
            }
        };
        jpeg = new Camera.PictureCallback() {
            @Override
            public void onPictureTaken(byte[] data, Camera camera) {

            }
        };
        cam.takePicture(shutterCallback, raw, jpeg);

        return false;
    }

    @Override
    public boolean capture() {
        return false;
    }

    @Override
    public Image getRaw() {
        return null;
    }

    @Override
    public Camera.ShutterCallback getCallback() {
        return null;
    }

    @Override
    public void setCallback(Camera.ShutterCallback call) {

    }
}
