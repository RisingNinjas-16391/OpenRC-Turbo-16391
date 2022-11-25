package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;


public class AutoCommand1 extends SequentialCommandGroup {
    public AutoCommand1(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, TurretSubsystem turret, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {
        TrajectorySequenceSupplier Trajectory1 = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(45)), Math.toRadians(60))
                .build();

        TrajectorySequenceSupplier Trajectory2 = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-28, -5, Math.toRadians(45))).setTangent(Math.toRadians(215))
                .splineToSplineHeading(new Pose2d(-62, -11.5, Math.toRadians(0)), Math.toRadians(180))
                .build();

        TrajectorySequenceSupplier Trajectory3 = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-62, -11.5,  Math.toRadians(45))).setTangent(0)
                .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(45)), Math.toRadians(30))
                .build();

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-32, -7,  Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-10, -34, Math.toRadians(90)))
                .build();

        TrajectorySequenceSupplier parkCenter = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-32, -7,  Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                .build();

        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(new Pose2d(-32, -7,  Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-58, -20,  Math.toRadians(90)))
                .build();

        TrajectorySequenceSupplier parkTrajectory = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(5)))
                .build();

        SequentialCommandGroup initToStack = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Part 1 Complete");
                    telemetry.update();
                }),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, Trajectory1).withTimeout(5000),
                    new LiftCommand(lift, 1),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.FEED)
                ),
                new LiftCommand(lift, 0).withTimeout(1000)
            );


        SequentialCommandGroup stackToHigh = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Part 2 Complete");
                    telemetry.update();
                }),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, Trajectory2).withTimeout(5000),
                    new TurretPositionCommand(turret, lift::getCurrentHeight, true),
                    new LiftCommand(lift, 4)),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.FEED)
                );

        SequentialCommandGroup highToStack = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Part 3 Complete");
                    telemetry.update();
                }),
                new ParallelCommandGroup(
                    new FollowTrajectoryCommand(drivetrain, Trajectory3).withTimeout(5000),
                    new TurretPositionCommand(turret, lift::getCurrentHeight, false),
                    new LiftCommand(lift, 1),
                    new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED)
                ),
                new LiftCommand(lift, 0).withTimeout(1000),
                new LiftCommand(lift, 1).withTimeout(1000)
            );


        addCommands(
                new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)))),
                new InstantCommand(aprilTagDetector::detect),
                initToStack,
                stackToHigh,
                highToStack,
                stackToHigh,
                // Park Left
                new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkTrajectory),
                        // Park Right
                        new ConditionalCommand(new FollowTrajectoryCommand(drivetrain, parkTrajectory),
                                // Park Center
                                new FollowTrajectoryCommand(drivetrain, parkTrajectory),
                                ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.RIGHT)
                        , ()-> aprilTagDetector.getParkLocation() == AprilTagSubsystem.Detection.LEFT)
        );
    }
}
