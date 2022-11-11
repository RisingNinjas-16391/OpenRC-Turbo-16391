package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.opmode.AutoConstants.threshold;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutoConstants.timeout;
import static org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_A;
import static org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_B;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


public abstract class AutonomousTemplate extends LinearOpMode {
    final AprilTagDetector aprilTagdetector = new AprilTagDetector(hardwareMap);

    @Override
    public void runOpMode() {
        initialize();
        telemetry.addData(">", "Ready when you are!");
        telemetry.update();
        Log.i("Robot", "standby");
        waitForStart();
        if (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections =  aprilTagdetector.detect(timeout, threshold);
            Map.Entry<Integer, Integer> maxDetection = null;
            if (detections.size() > 0) {
                Map<Integer, Integer> detectionsCount = new HashMap<>();
                for (AprilTagDetection d: detections) {
                    if(detectionsCount.containsKey(d.id)) {
                        detectionsCount.put(d.id, detectionsCount.get(d.id) + 1);
                    } else {
                        detectionsCount.put(d.id, 0);
                    }


                    for (Map.Entry<Integer, Integer> entry: detectionsCount.entrySet())
                    {
                        if (maxDetection == null || entry.getValue().compareTo(maxDetection.getValue()) > 0)
                        {
                            maxDetection = entry;
                        }
                    }
                }


            }

            regularAutonomous();

            switch(maxDetection.getKey()) {
                case TAG_ID_A: pathA();
                    Log.i("Robot", "auto a");
                    telemetry.addLine("auto a");
                    telemetry.update();
                    break;

                case TAG_ID_B: pathB();
                    Log.i("Robot", "auto b");
                    telemetry.addLine("auto b");
                    telemetry.update();
                break;

                default: pathC();
                    Log.i("Robot", "auto c");
                    telemetry.addLine("auto c");
                    telemetry.update();
                break;
            }


        }
    }

    public abstract void initialize();

    public abstract void pathA();

    public abstract void pathB();

    public abstract void pathC();

    public abstract void regularAutonomous();
}