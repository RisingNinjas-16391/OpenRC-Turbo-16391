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
    private final DoubleSupplier liftHeight;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public DrivetrainCommand(DrivetrainSubsystem drivetrain, DoubleSupplier liftHeight,
                             DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.liftHeight = liftHeight;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

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

        drivetrain.drive(forward.getAsDouble() * correction,
                -strafe.getAsDouble() * correction, -turn.getAsDouble() * correction);
    }
}
