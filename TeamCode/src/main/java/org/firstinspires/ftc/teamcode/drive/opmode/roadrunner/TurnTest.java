package org.firstinspires.ftc.teamcode.drive.opmode.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.MecanumDriveR;

/*
 * This is a simple routine to test turning capabilities.
 */
// @Disabled
// @Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveR drive = new MecanumDriveR(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
