package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;

import java.util.Locale;


public class AutoCommand1 extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();
    public AutoCommand1(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, TurretSubsystem turret, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {
        TrajectorySequenceSupplier initToHighTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(45)), Math.toRadians(60))
                .build();

        TrajectorySequenceSupplier highToStackTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45))).setTangent(Math.toRadians(215))
                .splineToSplineHeading(new Pose2d(-62, -11.5, Math.toRadians(0)), Math.toRadians(180))
                .build();

        TrajectorySequenceSupplier stackToHighTrajectory = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-62, -11.5,  Math.toRadians(0))).setTangent(0)
                .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(45)), Math.toRadians(30))
                .build();

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-37, -34, Math.toRadians(90)), Math.toRadians(270)).setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60, -34, Math.toRadians(90)), Math.toRadians(180))
                .build();

        TrajectorySequenceSupplier parkCenter = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-34, -34, Math.toRadians(90)), Math.toRadians(270))
                .build();

        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-30, -34, Math.toRadians(90)), Math.toRadians(-45)).setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -34, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequenceSupplier parkTrajectory = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(5)))
                .build();

        SequentialCommandGroup initToHigh = new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, initToHighTrajectory).withTimeout(5000),
                    new LiftCommand(lift, 4),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.FEED)
                )
            );

        SequentialCommandGroup highToStack = new SequentialCommandGroup(
                new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED),
                new WaitCommand(500),
                new ParallelDeadlineGroup(
                    new FollowTrajectoryCommand(drivetrain, highToStackTrajectory).withTimeout(5000),
                    new TurretPositionCommand(turret, lift::getCurrentHeight, false),
                    new LiftCommand(lift, 1),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.FEED)
                ),
                new LiftCommand(lift, 0).withTimeout(500)

            );

        SequentialCommandGroup stackToHigh = new SequentialCommandGroup(
                new LiftCommand(lift, 4).withTimeout(250),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, stackToHighTrajectory).withTimeout(5000),
                        new TurretPositionCommand(turret, lift::getCurrentHeight, true),
                        new LiftCommand(lift, 4)
                )
        );

        Command displayTime = new InstantCommand(() -> System.out.println(String.format("Time Left: %f.2", 30 - timer.time())));
        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));
                }),
                new InstantCommand(aprilTagDetector::detect),
                // Preload
                initToHigh,
                highToStack,
                new PrintCommand("Preload Stack"),
                displayTime,
                // 1
                stackToHigh,
                highToStack,
                new PrintCommand("First Stack"),
                displayTime,
                // 2
                stackToHigh,
                highToStack,
                new PrintCommand("Second Stack"),
                displayTime,
                // 3
                stackToHigh,
                highToStack,
                new PrintCommand("Third Stack"),
                displayTime,
                // 5
                stackToHigh,
                // Park Left
                new PrintCommand("Fourth Stack"),
                displayTime,
                new LiftCommand(lift, 0).withTimeout(1),
                new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkLeft),
                        // Park Right
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkRight),
                                // Park Center
                                new FollowTrajectoryCommand(drivetrain, parkCenter),
                                ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT),
                        ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT
                ),
                new PrintCommand(("Parked: " + aprilTagDetector.getParkLocation().toString())),
                displayTime
        );
    }
}
