package org.firstinspires.ftc.teamcode;

import android.graphics.Point;
import android.hardware.Camera;
//import android.media.MediaRecorder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
/**
 * Created by FTC on 11/14/2015.
 */
public class ExperimentMode extends OpMode {
    DcMotor l;
    DcMotor r;
    Camera cam; //Camera
    /*
    Camera.ShutterCallback scb = new Camera.ShutterCallback() {
        @Override
        public void onShutter() {

        }
    }; //Set callbacks
    Camera.PictureCallback pcb = new Camera.PictureCallback() {
        @Override
        public void onPictureTaken(byte[] data, Camera camera) {

        }
    };
    Camera.PictureCallback jpeg = new Camera.PictureCallback() {
        @Override
        public void onPictureTaken(byte[] data, Camera camera) {

        }
    };
    */
    Camera.Face face = new Camera.Face();
    //Point p = new Point();
    Camera.FaceDetectionListener fdl = new Camera.FaceDetectionListener() {
        @Override
        public void onFaceDetection(Camera.Face[] faces, Camera camera) {
            face = faces[0];
        }
    };
    @Override
    public void init() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        r.setDirection(DcMotor.Direction.REVERSE);
        cam = Camera.open(0);
        cam.startFaceDetection();
        cam.setFaceDetectionListener(fdl);
    }

    @Override
    public void loop() {
        if (face.mouth.x > 500){
            l.setPower(1.0);
            r.setPower(-1.0);
        } else if (face != null) {
            l.setPower(-1.0);
            r.setPower(1.0);
        } else {
            l.setPower(0.0);
            r.setPower(0.0);
        }
        telemetry.addData("Face x:", face.mouth.x);
        telemetry.addData("Face y", face.mouth.y);
    }
}
