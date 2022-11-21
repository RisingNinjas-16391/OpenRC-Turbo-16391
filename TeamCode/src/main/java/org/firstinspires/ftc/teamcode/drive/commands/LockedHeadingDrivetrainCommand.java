package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.MecanumDrive.HEADING_PID;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class LockedHeadingDrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier liftHeight;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final PIDFController pidController = new PIDFController(HEADING_PID);

    public LockedHeadingDrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier liftHeight,
                                          DoubleSupplier forward, DoubleSupplier strafe, double turn) {
        this.drivetrain = drivetrain;
        this.liftHeight = liftHeight;
        this.forward = forward;
        this.strafe = strafe;
        pidController.setTargetPosition(turn);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double correction;
        if (liftHeight.getAsDouble() > 0) {
            correction = 1 - (liftHeight.getAsDouble() / 10);
            correction /= 2;
            correction += 0.5;;
        } else {
            correction = 1;
        }

        // Read pose
        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -forward.getAsDouble() * correction,
                -strafe.getAsDouble() * correction
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately

        drivetrain.drive(input.getX(), input.getY(), pidController.update(poseEstimate.getHeading()));
    }
}
