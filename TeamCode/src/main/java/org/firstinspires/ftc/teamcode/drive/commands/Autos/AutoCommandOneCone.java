package org.firstinspires.ftc.teamcode.drive.commands.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;


public class AutoCommandOneCone extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();

    public AutoCommandOneCone(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, AprilTagSubsystem aprilTagDetector, boolean right) {

        Command initToStack = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, AutoTrajectories.initToStackTrajectory(drivetrain, right)),
                        new SequentialCommandGroup(
                                new WaitCommand(2000),
                                new IntakeCommand(intake, IntakeSubsystem.Direction.FEED),
                                new LiftCommand(lift, 2))
                ),
                new WaitCommand(500),
                new LiftCommand(lift, 6.2).withTimeout(500),
                new WaitCommand(500)
        );

        Command stackToHigh = new SequentialCommandGroup(
                new LiftCommand(lift, 2),
                new ParallelCommandGroup(
                        new LiftCommand(lift, 4),
                        new FollowTrajectoryCommand(drivetrain, AutoTrajectories.stackToHighTrajectory(drivetrain, right))
                ),
                new LiftCommand(lift, 3),
                new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED),
                new WaitCommand(500),
                new LiftCommand(lift, 4),
                new WaitCommand(500)
        );

        Command highToPark = new ParallelCommandGroup(
                new LiftCommand(lift, 0),
                new FollowTrajectoryCommand(drivetrain, AutoTrajectories.highToParkTrajectory(drivetrain, right))
        );

        Command displayTime = new InstantCommand(() -> System.out.printf("Time Left: %f.2%n", 30 - timer.time()));


        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(Waypoints.INIT);
                }),

                new InstantCommand(aprilTagDetector::detect),

                initToStack,
                stackToHigh,
                highToPark,

                new ParallelCommandGroup(
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, AutoTrajectories.parkLeft(drivetrain, right)),
                                // Park Right
                                new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, AutoTrajectories.parkRight(drivetrain, right)),
                                        // Park Center
                                        new InstantCommand(),
                                        () -> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT),
                                () -> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT
                        )
                ),

                new PrintCommand(("Parked: " + aprilTagDetector.getParkLocation().toString())),
                displayTime
        );
    }
}
