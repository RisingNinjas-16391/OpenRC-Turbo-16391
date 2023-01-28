package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;


public class AutoCommandOneCone extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();

    public AutoCommandOneCone(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {

        TrajectorySequenceSupplier initToCone1 = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate()).setTangent(90)
                .strafeTo(new Vector2d(35, -25))
                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(5)).setTangent(Math.toRadians(60)).setTangent(Math.toRadians(0))
                .build();

        TrajectorySequenceSupplier coneToHigh = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate()).setTangent(90)
                .strafeTo(new Vector2d(43, -12))
                .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(135)), Math.toRadians(140)).setTangent(Math.toRadians(315))
                .build();

        TrajectorySequenceSupplier hightoPark = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate()).setTangent(300)
                .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(90)), Math.toRadians(270))
                .build();

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate()).setTangent(90)
                .strafeLeft(25)
                .build();

        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate()).setTangent(90)
                .strafeRight(25)
                .build();


        Command displayTime = new InstantCommand(() -> System.out.printf("Time Left: %f.2%n", 30 - timer.time()));
        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(new Pose2d(35, -62, Math.toRadians(90)));
                }),
                new InstantCommand(aprilTagDetector::detect),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drivetrain, initToCone1),
                        new LiftCommand(lift, LiftConstants.LOW_POS)
                ),
                new LiftCommand(lift, 5), //TODO: set to height of auto cones
                new LiftCommand(lift, LiftConstants.LOW_POS),
                new ParallelDeadlineGroup(
                    new FollowTrajectoryCommand(drivetrain, coneToHigh),
                    new LiftCommand(lift, LiftConstants.HIGH_POS)
                ),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(500),
                        new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED)
                    ),
                    new LiftCommand(lift, LiftConstants.BOTTOM_POS)
                ),
                new FollowTrajectoryCommand(drivetrain, hightoPark),
                new ParallelDeadlineGroup(
                    new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkLeft),
                        // Park Right
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkRight),
                                // Park Center
                                new InstantCommand(),
                                ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT),
                        ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT
                    ),
                    new LiftCommand(lift, LiftConstants.FEED_POS)
                ),

                new PrintCommand(("Parked: " + aprilTagDetector.getParkLocation().toString())),
                displayTime
        );
    }
}
