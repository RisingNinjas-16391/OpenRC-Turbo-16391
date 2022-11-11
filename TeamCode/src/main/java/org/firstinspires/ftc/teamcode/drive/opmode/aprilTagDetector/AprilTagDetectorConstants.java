package org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector;

public class AprilTagDetectorConstants {
    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public final static double fx = 578.272;
    public final static double fy = 578.272;
    public final static double cx = 402.145;
    public final static double cy = 221.506;

    // UNITS ARE METERS
    public final static double TAGSIZE = 0.166;

    public final static float DECIMATION_HIGH = 3;
    public final static float DECIMATION_LOW = 2;
    public final static float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    public final static int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public final static int TAG_ID_LEFT = 0;
    public final static int TAG_ID_CENTER = 1;
    public final static int TAG_ID_RIGHT = 2;
}
