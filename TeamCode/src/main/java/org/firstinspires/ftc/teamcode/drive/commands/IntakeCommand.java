package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intake;
    private BooleanSupplier feed;

    public IntakeCommand(IntakeSubsystem intake, BooleanSupplier feed) {
        this.intake = intake;
        this.feed = feed;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (feed.getAsBoolean()) {
            intake.unfeed();
        } else {
            intake.feed();
        }
    }
}
