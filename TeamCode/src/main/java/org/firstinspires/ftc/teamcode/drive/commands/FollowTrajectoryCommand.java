package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final TrajectorySequenceSupplier trajectory;


    public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, TrajectorySequenceSupplier trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.runTrajectory(trajectory.getAsTrajectorySequence());
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.isBusy();
    }
}
