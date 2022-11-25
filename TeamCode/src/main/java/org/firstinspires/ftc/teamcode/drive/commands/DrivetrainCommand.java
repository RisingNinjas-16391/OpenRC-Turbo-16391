package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.helpers.DoubleSupplierSupplier;

import java.util.function.DoubleSupplier;

public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplierSupplier multiplier;
    private final DoubleSupplierSupplier forward;
    private final DoubleSupplierSupplier strafe;
    private final DoubleSupplierSupplier turn;

    public DrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplierSupplier multiplier,
                             DoubleSupplierSupplier forward, DoubleSupplierSupplier strafe, DoubleSupplierSupplier turn) {
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
                forward.getAsDoubleSupplier().getAsDouble(),
                -strafe.getAsDoubleSupplier().getAsDouble(),
                -turn.getAsDoubleSupplier().getAsDouble(),
                multiplier.getAsDoubleSupplier().getAsDouble());
    }
}
