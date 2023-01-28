package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Class containing constants used for {@link LiftSubsystem}
 */
@Config
public final class LiftConstants {
    // if necessary, reverse the motor so "up" is positive
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    // public static final int maxHeight = 4500; // Ticks for fully-extended lift


    /**
     * Inches for lift presets
     */
    public static double idk = 1.5;
    public static double BOTTOM_POS = 0;
    public static double FEED_POS = 8 +idk;
    public static double LOW_POS = 15 +idk;
    public static double MID_POS = 25 +idk;
    public static double HIGH_POS = 32 + idk;
    public static double EX_HIGH_POS = 41 +idk ;
    public static double Inches_above = 3;
    public static double SCORE_ADJ = 7;


    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    public static double heightThreshold = 0.5;
    public static boolean simpleMode = false;
    public static final String name = "slide";
    public static double minHeightTurret = 5;
    public static final double SPOOL_RADIUS = 0.75; // in
    public static final double GEAR_RATIO = 5; // output (spool) speed / input (motor) speed
    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]n
    public static final double MAX_HEIGHT = 37; // inches
    public static double MAX_VEL = 600; // in/s
    public static double MAX_ACCEL = 600; // in/s^2
    public static double MAX_JERK = 400; // in/s^3
    public static PIDCoefficients kPID = new PIDCoefficients(2.35, 0, 0);
    public static double kV = 0.01;
    public static double kA = 0.001;
    public static double kG = 0.001;
    public static double kStatic = 0.1;

}
