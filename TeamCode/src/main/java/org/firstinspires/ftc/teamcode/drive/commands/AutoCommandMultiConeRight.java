package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;


public class AutoCommandMultiConeRight extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();

    public AutoCommandMultiConeRight(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {

        TrajectorySequenceSupplier initToStackTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(35, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(-35, -25))
                .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(190)).setTangent(Math.toRadians(60)).setTangent(Math.toRadians(0))
                .build();

        TrajectorySequenceSupplier stackToHighTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(60, -12, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, -12))
                .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(135)), Math.toRadians(140)).setTangent(Math.toRadians(315))
                .build();


        TrajectorySequenceSupplier highToStackTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(28, -5, Math.toRadians(315)))
                .splineToSplineHeading(new Pose2d(43, -12, Math.toRadians(0)), Math.toRadians(0))
                //.setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(60, -12))
                .build();

        TrajectorySequenceSupplier highToParkTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(28, -5, Math.toRadians(300)))
                .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(270))
                .build();

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(35, -35, Math.toRadians(45)))
                .strafeTo(new Vector2d(16, -35))
                .build();


        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(35, -35, Math.toRadians(45)))
                .strafeTo(new Vector2d(60, -35))
                .build();
        TrajectorySequenceSupplier parkCenter = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(35, -35, Math.toRadians(45)))
                .strafeTo(new Vector2d(35, -34))
                .build();

        SequentialCommandGroup initToStack = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, initToStackTrajectory).withTimeout(5000)
//                        new LiftCommand(lift, 2),
//                        new IntakeCommand(intake, IntakeSubsystem.Direction.FEED)
                )
        );

        SequentialCommandGroup stackToHigh = new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new LiftCommand(lift, 4),
                        new FollowTrajectoryCommand(drivetrain, stackToHighTrajectory).withTimeout(5000)
//                        new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED)
                )
        );

        SequentialCommandGroup highToPark = new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new LiftCommand(lift, 4),
                        new FollowTrajectoryCommand(drivetrain, highToParkTrajectory).withTimeout(5000)
//                        new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED)
                )
        );
        SequentialCommandGroup highToStack = new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new LiftCommand(lift, 4),
                        new FollowTrajectoryCommand(drivetrain, highToStackTrajectory).withTimeout(5000)
//                        new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED)
                )
        );

        Command displayTime = new InstantCommand(() -> System.out.printf("Time Left: %f.2%n", 30 - timer.time()));
        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(new Pose2d(35, -62, Math.toRadians(90)));
                }),

                new InstantCommand(aprilTagDetector::detect),
                initToStack,
//                new LiftCommand(lift, 0),
                stackToHigh,
                highToStack,
                stackToHigh,
                highToPark,
                new ParallelCommandGroup(
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkLeft),
                                // Park Right
                                new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkRight),
                                        // Park Center
                                        new FollowTrajectoryCommand(drivetrain, parkCenter),
                                        () -> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT),
                                () -> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT
                        )
                ),

                new PrintCommand(("Parked: " + aprilTagDetector.getParkLocation().toString())),
                displayTime
        );
    }
}
