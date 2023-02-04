package org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagDetector {
    int numFramesWithoutDetection = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    ElapsedTime timer = new ElapsedTime();
    // If there's been a new frame...

    public AprilTagDetector(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(AprilTagDetectorConstants.TAGSIZE, AprilTagDetectorConstants.fx, AprilTagDetectorConstants.fy, AprilTagDetectorConstants.cx, AprilTagDetectorConstants.cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public ArrayList<AprilTagDetection> detect(double timeout, int threshold) {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        timer.reset();
        while (detections.size() < threshold && timer.time() < timeout) {
            detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            Log.i("Robot Detection FPS", String.format("%f", camera.getFps()));
            Log.i("Robot Detection Overhead ms", String.format("%d", camera.getOverheadTimeMs()));
            Log.i("Robot Detection Pipeline ms", String.format("%d", camera.getPipelineTimeMs()));

            // If we don't see any tags
            numFramesWithoutDetection++;

            // If we haven't seen a tag for a few frames, lower the decimation
            // so we can hopefully pick one up if we're e.g. far back
            if (numFramesWithoutDetection >= AprilTagDetectorConstants.THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                aprilTagDetectionPipeline.setDecimation(AprilTagDetectorConstants.DECIMATION_LOW);
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                if (detections == null) {
                    return new ArrayList<>();
                }

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < AprilTagDetectorConstants.THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(AprilTagDetectorConstants.DECIMATION_HIGH);
                }
            }
        }
        return detections;
    }
}
