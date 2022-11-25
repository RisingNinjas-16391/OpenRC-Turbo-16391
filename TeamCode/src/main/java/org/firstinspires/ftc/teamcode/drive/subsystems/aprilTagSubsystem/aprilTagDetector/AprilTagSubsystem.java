package org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector;

import static org.firstinspires.ftc.teamcode.drive.opmode.opmodes.AutoConstants.threshold;
import static org.firstinspires.ftc.teamcode.drive.opmode.opmodes.AutoConstants.timeout;
import static org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_CENTER;
import static org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_LEFT;
import static org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_RIGHT;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagSubsystem extends SubsystemBase {
    private AprilTagDetector aprilTagDetector;
    private Telemetry telemetry;
    private Detection savedDetection;
    public enum Detection {
        LEFT, CENTER, RIGHT, NONE
    }

    public AprilTagSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        aprilTagDetector = new AprilTagDetector(hwMap);
        this.telemetry = telemetry;

    }

    public void detect() {
        ArrayList<AprilTagDetection> detections =  aprilTagDetector.detect(timeout, threshold);
        if (detections.size() > 0) {
            switch (detections.get(0).id) {
                case TAG_ID_LEFT:
                    Log.i("Robot", "auto a");
                    telemetry.addLine("auto a");
                    telemetry.update();
                    savedDetection = Detection.LEFT;
                    return;

                case TAG_ID_CENTER:
                    Log.i("Robot", "auto b");
                    telemetry.addLine("auto b");
                    telemetry.update();
                    savedDetection = Detection.CENTER;
                    return;

                case TAG_ID_RIGHT:
                    Log.i("Robot", "auto c");
                    telemetry.addLine("auto c");
                    telemetry.update();
                    savedDetection = Detection.RIGHT;
                    return;
            }
            telemetry.addLine("No park auto");
            telemetry.update();
            savedDetection = Detection.NONE;
        }

    }
    public Detection getParkLocation() {
        return savedDetection;
    }



}
