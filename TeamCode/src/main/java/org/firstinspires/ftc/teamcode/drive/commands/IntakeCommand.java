package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final IntakeSubsystem.Direction direction;
    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.Direction direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntake(direction);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
