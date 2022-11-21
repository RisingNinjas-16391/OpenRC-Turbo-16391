package org.firstinspires.ftc.teamcode.drive.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.aprilTagDetector.AprilTagDetector;
import org.firstinspires.ftc.teamcode.drive.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoCommand1 extends SequentialCommandGroup {
    public AutoCommand1(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, AprilTagSubsystem aprilTagDetector) {
        TrajectorySequence Trajectory1 = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                .build();

        TrajectorySequence Trajectory2 = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-38, -12,  Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-32, -7,  Math.toRadians(45)))
                .build();

        TrajectorySequence Trajectory3 = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-38, -12,  Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-58, -12,  Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence parkLeft = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-10, -34, Math.toRadians(90)))
                .build();

        TrajectorySequence parkCenter = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                .build();

        TrajectorySequence parkRight = drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-58, -20,  Math.toRadians(90)))
                .build();

        SequentialCommandGroup stackToHigh = new SequentialCommandGroup(new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.runTrajectory(Trajectory2)).withTimeout(5000),
                new LiftCommand(lift, () -> 4)),
                new IntakeCommand(intake, () -> false).withTimeout(500));

        SequentialCommandGroup highToStack = new SequentialCommandGroup(new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.runTrajectory(Trajectory3)).withTimeout(5000),
                new LiftCommand(lift, () -> 1),
                new IntakeCommand(intake, () -> true)),
                new LiftCommand(lift, () -> 0).withTimeout(1000),
                new LiftCommand(lift, () -> 1).withTimeout(1000)
        );
        final TrajectorySequence parkTrajectory;
        switch (aprilTagDetector.getParkLocation()) {
            case 0:

            case 1:
                parkTrajectory = parkLeft;
                break;
            case 2:
                parkTrajectory = parkCenter;
                break;
            case 3:
                parkTrajectory = parkRight;
                break;

        }
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drivetrain.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)))),
                    new InstantCommand(() -> drivetrain.runTrajectory(Trajectory1)).withTimeout(5000),
                    new LiftCommand(lift, () -> 1),
                    new IntakeCommand(intake, () -> true)),
                new LiftCommand(lift, () -> 0).withTimeout(1000),
                new LiftCommand(lift, () -> 1).withTimeout(1000),
                stackToHigh,
                highToStack,
                stackToHigh,
                highToStack,
                new InstantCommand(() -> drivetrain.runTrajectory(parkTrajectory))
        );
    }
}
