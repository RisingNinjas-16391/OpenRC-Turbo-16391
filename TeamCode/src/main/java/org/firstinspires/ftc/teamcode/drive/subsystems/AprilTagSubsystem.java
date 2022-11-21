package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutoConstants.threshold;
import static org.firstinspires.ftc.teamcode.drive.opmode.AutoConstants.timeout;
import static org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetectorConstants.TAG_ID_RIGHT;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagSubsystem extends SubsystemBase {
    private AprilTagDetector aprilTagDetector;
    private Telemetry telemetry;

    public AprilTagSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        aprilTagDetector = new AprilTagDetector(hwMap);
        this.telemetry = telemetry;

    }

    public int getParkLocation() {
        ArrayList<AprilTagDetection> detections =  aprilTagDetector.detect(timeout, threshold);

        switch (detections.get(0).id) {
            case TAG_ID_LEFT:
                Log.i("Robot", "auto a");
                telemetry.addLine("auto a");
                telemetry.update();
                return 1;

            case TAG_ID_CENTER:
                Log.i("Robot", "auto b");
                telemetry.addLine("auto b");
                telemetry.update();
                return 2;

            case TAG_ID_RIGHT:
                Log.i("Robot", "auto c");
                telemetry.addLine("auto c");
                telemetry.update();
                return 3;

            default:
                telemetry.addLine("No park auto");
                telemetry.update();
                return 0;
        }
    }



}
