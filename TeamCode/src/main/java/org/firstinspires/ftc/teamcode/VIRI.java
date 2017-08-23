package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Camera;
import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * VIRI
 *
 * Virtual interactive Robotic Intelligence
 *
 * VIRI is an interactive experience for outreach events, designed
 * to interact as a human would with kids. So far, all it does
 * is find a person's face and say "Hello; I am VIRI" but will
 * be developed further when the time is available.
 */
public class VIRI extends OpMode {
    /*
        VIRI

        Virtual Interactive Robotic Intelligence
     */
    DcMotor l;
    DcMotor r;
    Camera cam; //Camera
    TextToSpeech tts;
    TextToSpeech.OnInitListener ttsListener;
    Context c;
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
        c = c.getApplicationContext();
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        r.setDirection(DcMotor.Direction.REVERSE);
        cam = Camera.open(0);
        cam.startFaceDetection();
        cam.setFaceDetectionListener(fdl);
        ttsListener = new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                tts = new TextToSpeech(c, ttsListener);
            }
        };
    }

    @Override
    public void loop() {
        // TODO add better face tracking
        while (face.mouth.x > 505 && face.mouth.x < 495) {
            if (face.mouth.x > 505) {
                l.setPower(1.0);
                r.setPower(-1.0);
            } else if (face.mouth.x < 495) {
                l.setPower(-1.0);
                r.setPower(1.0);
            }
            telemetry.addData("Face x:", face.mouth.x);
            telemetry.addData("Face y", face.mouth.y);
        }
        // TODO Add "drive up to person"
        l.setPower(0.0);
        r.setPower(0.0);
        // TODO find out what "VIRI" sounds like on tts
        talk("Hello. I am VIRI.");
        talk("Also known as virtual interactive robotic intelligence.");
    }

    /**
     * Calls the tts engine in a simplified manner.
     *
     * @param words
     */
    private void talk(String words) {
        tts.speak(words, TextToSpeech.QUEUE_ADD, null);
    }
}
