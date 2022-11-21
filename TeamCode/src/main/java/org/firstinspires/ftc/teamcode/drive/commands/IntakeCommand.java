package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final boolean direction;
    public IntakeCommand(IntakeSubsystem intake, boolean direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (direction) {
            intake.feed();
        }
        else {
            intake.unfeed();
        }
    }
}
