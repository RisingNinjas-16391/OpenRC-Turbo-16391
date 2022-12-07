package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldCentricDrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier multiplier;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public FieldCentricDrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier multiplier,
                                         DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.multiplier = multiplier;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        // Read pose
        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                forward.getAsDouble(),
                -strafe.getAsDouble()
        ).rotated(-poseEstimate.getHeading() + Math.toRadians(180));

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drivetrain.driveMultiplied(
                -input.getX(),
                -input.getY(),
                -turn.getAsDouble(),
                multiplier.getAsDouble());
    }
}
