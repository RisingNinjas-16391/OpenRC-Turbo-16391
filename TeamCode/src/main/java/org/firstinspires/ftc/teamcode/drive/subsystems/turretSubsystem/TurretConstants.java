package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Class containing constants used for {@link TurretSubsystem}
 */
@Config
public final class TurretConstants {
    public static final DcMotorSimple.Direction rotationDirection = DcMotorSimple.Direction.FORWARD;

    public static final int tickMargin = 1; // Margin for getBusy() false

    /**
     * Ticks for lift presets
     */
    public static final int homePos = 0;
    public static final int otherSidePos = 144; // 288 ticks for full revolution
    public static final String name = "turret";
    public static PIDFCoefficients kPosPID = new PIDFCoefficients(0.005, 0, 0, 0);

}
