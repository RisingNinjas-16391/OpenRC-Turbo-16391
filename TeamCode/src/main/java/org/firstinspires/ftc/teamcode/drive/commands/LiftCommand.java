package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final int heightIndex;
    private final double height;

    public LiftCommand(LiftSubsystem lift, int heightIndex) {
        this.lift = lift;
        this.heightIndex = heightIndex;
        this.height = 0;
        addRequirements(lift);
    }

    public LiftCommand(LiftSubsystem lift, double height) {
        this.lift = lift;
        this.heightIndex = -1;
        this.height = height;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (heightIndex == -1) {
            lift.setHeight(height);
        } else {
            lift.indexToHeight(heightIndex);
        }
    }

    @Override
    public boolean isFinished() {
        return !lift.isBusy();
    }
}

