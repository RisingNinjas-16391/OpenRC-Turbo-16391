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


public class AutoCommand1 extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();
    final double parkY = -40;
    final Pose2d initPosition = new Pose2d(-35, -60, Math.toRadians(90));
    final Pose2d highPosition = new Pose2d(-28, -4.75, Math.toRadians(45));
    final Pose2d stackPosition = new Pose2d(-60.5, -11.25, Math.toRadians(0));
    double stackHeight = 3.0;
    final double stackIncrement = 0.4;
    final DrivetrainSubsystem drivetrain;
    final LiftSubsystem lift;
    final IntakeSubsystem intake;
    final TurretSubsystem turret;
    final TrajectorySequenceSupplier highToStackTrajectory;
    public AutoCommand1(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, TurretSubsystem turret, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.lift = lift;
        this.intake = intake;
        this.turret = turret;
        TrajectorySequenceSupplier initToHighTrajectory = () -> drivetrain.trajectorySequenceBuilder(initPosition)
                .splineToSplineHeading(highPosition, Math.toRadians(60))
                .build();

        highToStackTrajectory = () -> drivetrain.trajectorySequenceBuilder(highPosition).setTangent(Math.toRadians(215))
                .splineToSplineHeading(stackPosition, Math.toRadians(180))
                .build();

        TrajectorySequenceSupplier stackToHighTrajectory = () -> drivetrain.trajectorySequenceBuilder(stackPosition).setTangent(0)
                .splineToSplineHeading(highPosition, Math.toRadians(30))
                .build();

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-37, parkY, Math.toRadians(90)), Math.toRadians(270)).setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, parkY, Math.toRadians(90)), Math.toRadians(180))
                .build();

        TrajectorySequenceSupplier parkCenter = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-34, parkY, Math.toRadians(90)), Math.toRadians(270))
                .build();

        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-30, parkY, Math.toRadians(90)), Math.toRadians(-45)).setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, parkY, Math.toRadians(90)), Math.toRadians(0))
                .build();

        SequentialCommandGroup initToHigh = new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, initToHighTrajectory).withTimeout(5000),
                    new LiftCommand(lift, 34.0),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.FEED).perpetually().withTimeout(100)
                )
            );


        SequentialCommandGroup stackToHigh = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new LiftCommand(lift, 34.0),
                        new FollowTrajectoryCommand(drivetrain, stackToHighTrajectory).withTimeout(5000),
                        new TurretPositionCommand(turret, lift::getCurrentHeight, true)
                )
        );

        Command displayTime = new InstantCommand(() -> System.out.printf("Time Left: %f.2%n", 30 - timer.time()));
        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(initPosition);
                }),
                new InstantCommand(aprilTagDetector::detect),
                // Preload
                initToHigh,
                getHighToStack(),
                new PrintCommand("Preload Stack"),
                displayTime,
                // 1
                stackToHigh,
                getHighToStack(),
                new PrintCommand("First Stack"),
                displayTime,
                // 2
                stackToHigh,
                getHighToStack(),
                new PrintCommand("Second Stack"),
                displayTime,
                // 3
                stackToHigh,
                // Park Left
                new PrintCommand("Third Stack"),
                displayTime,
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LiftCommand(lift, 0)
                        ),
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkLeft),
                                // Park Right
                                new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkRight),
                                        // Park Center
                                        new FollowTrajectoryCommand(drivetrain, parkCenter),
                                        ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT),
                                ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT
                        )
                ),

                new PrintCommand(("Parked: " + aprilTagDetector.getParkLocation().toString())),
                displayTime
        );
    }
    private Command getHighToStack() {
        Command highToStack = new SequentialCommandGroup(
                new WaitCommand(500),
                new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED).perpetually().withTimeout(1000),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(drivetrain, highToStackTrajectory).withTimeout(5000),
                        new TurretPositionCommand(turret, lift::getCurrentHeight, false),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LiftCommand(lift, stackHeight + 3.0)
                        ),
                        new IntakeCommand(intake, IntakeSubsystem.Direction.FEED).perpetually()
                ),
                new LiftCommand(lift, stackHeight)

        );
        stackHeight -= stackIncrement;
        return highToStack;
    }
}
