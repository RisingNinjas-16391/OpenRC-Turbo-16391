package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private final LiftSubsystem lift;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier turn;
    private BooleanSupplier slowMode;
    private BooleanSupplier turbo;

    public DrivetrainCommand(DrivetrainSubsystem drivetrain, LiftSubsystem lift,
                             DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn,
                             BooleanSupplier slowMode, BooleanSupplier turbo) {
        this.drivetrain = drivetrain;
        this.lift = lift;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.slowMode = slowMode;
        this.turbo = turbo;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double driveMultiplier;
        if (slowMode.getAsBoolean()) {
            driveMultiplier = 0.4;
        } else if (turbo.getAsBoolean()) {
            driveMultiplier = 1;
        } else {
            driveMultiplier = 0.7;
        }

        if (lift.getCurrentHeight() > 0) {
            double correction = 1 - (lift.getCurrentHeight() / 10);
            correction /= 2;
            correction += 0.5;
            driveMultiplier *= correction;
        }

        drivetrain.drive(forward.getAsDouble() * driveMultiplier,
                strafe.getAsDouble() * driveMultiplier, turn.getAsDouble() * driveMultiplier);
    }
}
