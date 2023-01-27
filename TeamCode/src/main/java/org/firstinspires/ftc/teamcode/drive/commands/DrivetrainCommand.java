package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.helpers.DoubleSupplierSupplier;

import java.util.function.DoubleSupplier;

public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier multiplier;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public DrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier multiplier,
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

        drivetrain.driveMultiplied(
                forward.getAsDouble(),
                -strafe.getAsDouble(),
                -turn.getAsDouble(),
                multiplier.getAsDouble());
    }
}

