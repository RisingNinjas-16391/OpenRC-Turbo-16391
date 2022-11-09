package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

/** Class containing constants used for {@link LiftSubsystem}*/
@Config
public final class LiftConstants {
    public static final int endMargin = 10;
    public static final int maxHeight = 4500; // Ticks for fully-extended lift
    public static PIDCoefficients kPosPID =  new PIDCoefficients(0.005, 0, 0);
    public static double kV = 0;
    public static double kA = 0;
    public static double kS = 0.05;
    public static double kG = 0.05;
    public static final String name = "slide";
}
