package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/** Class containing constants used for {@link LiftSubsystem}*/
@Config
public final class LiftConstants {
    // if necessary, reverse the motor so "up" is positive
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    // public static final int maxHeight = 4500; // Ticks for fully-extended lift


    /** Ticks for lift presets */
    public static final int BOTTOM_POS = 0;
    public static final int FEED_POS = 1000;
    public static final int LOW_POS = 1800;
    public static final int MID_POS = 3000;
    public static final int HIGH_POS = 4020;

    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    public static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double SPOOL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 10; // in

    public static double MAX_VEL = 10; // in/s
    public static double MAX_ACCEL = 10; // in/s^2
    public static double MAX_JERK = 20; // in/s^3

    public static PIDCoefficients kPID =  new PIDCoefficients(0.005, 0, 0);
    public static double kV = 0;
    public static double kA = 0;
    public static double kG = 0.05;
    public static double kStatic = 0.05;
    public static final String name = "slide";

}
