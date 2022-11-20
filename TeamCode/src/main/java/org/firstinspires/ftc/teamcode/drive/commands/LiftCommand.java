package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class LiftCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LiftSubsystem lift;
    private DoubleSupplier height;

    public LiftCommand(LiftSubsystem lift, DoubleSupplier height) {
        this.lift = lift;
        this.height = height;
    }

    @Override
    public void execute() {
        lift.setHeight(height.getAsDouble());
    }
}
