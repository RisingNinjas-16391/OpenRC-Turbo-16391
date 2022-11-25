package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final int heightIndex;


    public LiftCommand(LiftSubsystem lift, int heightIndex) {
        this.lift = lift;
        this.heightIndex = heightIndex;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.indexToHeight(heightIndex);
    }

    @Override
    public boolean isFinished() {
        return lift.isBusy();
    }
}

