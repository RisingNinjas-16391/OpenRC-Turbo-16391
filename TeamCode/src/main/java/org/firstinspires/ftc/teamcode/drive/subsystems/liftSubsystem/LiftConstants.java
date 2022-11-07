package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

/** Class containing constants used for {@link LiftSubsystem}*/
public final class LiftConstants {
    static final int maxHeight = 1000; // Ticks for fully-extended lift
    static final int endMargin = 10;
    static final PIDCoefficients kPosPID =  new PIDCoefficients(0, 0, 0);
    static final double kV = 0;
    static final double kA = 0;
    static final double kS = 0;
    static final double kG = 0;
    static final String name = "Linear Slide";
}
