package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Class containing constants used for {@link TurretSubsystem}
 */
// @Config
public final class TurretConstants {
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class);
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static final int tickMargin = 1; // Margin for getBusy() false

    /**
     * Ticks for lift presets
     */

    public static final int homePos = 0;
    public static final int otherSidePos = 125; // 288 ticks for full revolution
    public static final String name = "turret";
    public static final double kV = 0.02;
    public static final double kA = 0.0025;
    public static final double kStatic = 0.12;
    public static PIDCoefficients kPID = new PIDCoefficients(0.075, 0, 0);
    public static double MAX_VEL = 3000;
    public static double MAX_ACCEL = 3000;
    public static double MAX_JERK = 1500;

}
