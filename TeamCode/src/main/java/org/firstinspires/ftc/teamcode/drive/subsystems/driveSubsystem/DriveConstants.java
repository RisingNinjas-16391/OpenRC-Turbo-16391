package org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Class containing constants used for {@link MecanumDriveR}
 */
@Config
public final class DriveConstants {

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(18, 0, 6.5, 14);

    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.19; // in


    public static double LATERAL_MULTIPLIER = 1.2;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;
    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 50;
    public static double MAX_ANG_VEL = Math.toRadians(90);
    public static double MAX_ANG_ACCEL = Math.toRadians(70);

    public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = MecanumDriveR.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = MecanumDriveR.getAccelerationConstraint(MAX_ACCEL);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7, 0, 0);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}