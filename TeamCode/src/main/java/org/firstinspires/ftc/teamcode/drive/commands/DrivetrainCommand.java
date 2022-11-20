package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class DrivetrainCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem drivetrain;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier turn;

    public DrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    @Override
    public void execute() {
        drivetrain.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
    }
}
