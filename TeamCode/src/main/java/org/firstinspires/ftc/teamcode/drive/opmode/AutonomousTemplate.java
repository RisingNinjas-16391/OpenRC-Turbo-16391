package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class AutonomousTemplate extends LinearOpMode {
    List<Recognition> updatedRecognitions;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tensorflow;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tensorflow != null) {
            tensorflow.activate();

            tensorflow.setZoom(1.25, 2.5);
        }
        initialize();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        Log.i("Robot", "standby");
        waitForStart();
        if (opModeIsActive()) {
            int path = 3;
            starting();
            if (tensorflow != null) {
                updatedRecognitions = tensorflow.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Duck")) {
                            if (recognition.getLeft() <= 400) {
                                path = 1;
                            } else {
                                path = 2;
                            }
                        }
                        Log.i("Robot", String.format("Duck found at %f", recognition.getLeft()));
                    }
                    telemetry.update();
                }
            }

            if (tensorflow != null) {
                tensorflow.deactivate();
            }
            vuforia.close();

            switch(path) {
                case 1: leftPath();
                break;

                case 2: rightPath();
                break;

                default: unseenPath();
                break;
            }

            regularAutonomous();
        }
    }

    public void initialize() {
        telemetry.addLine("SetUp");
        telemetry.update();
        Log.i("Robot", "initializing");
    }

    public void starting() {
        telemetry.addLine("SetUp");
        telemetry.update();
        Log.i("Robot", "starting");

    }

    public void leftPath() {
        telemetry.addLine("leftPath");
        telemetry.update();
        Log.i("Robot", "left path");

    }

    public void rightPath() {
        telemetry.addLine("right path");
        telemetry.update();
        Log.i("Robot", "right path");

    }

    public void unseenPath() {
        telemetry.addLine("unseen path");
        telemetry.update();
        Log.i("Robot", "unseen path");

    }

    public void regularAutonomous() {
        telemetry.addLine("Regular Autonomous");
        telemetry.update();
        Log.i("Robot", "regular path");
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AutoConstants.VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.67f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tensorflow = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tensorflow.loadModelFromAsset(AutoConstants.TENSORFLOW_MODEL_ASSET, AutoConstants.LABELS);

    }
}