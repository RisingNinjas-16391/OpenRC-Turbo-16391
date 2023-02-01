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
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;


public class AutoCommandPark extends SequentialCommandGroup {
    ElapsedTime timer = new ElapsedTime();
    final double parkY = -41;

    public AutoCommandPark(DrivetrainSubsystem drivetrain, LiftSubsystem lift, IntakeSubsystem intake, AprilTagSubsystem aprilTagDetector, Telemetry telemetry) {

        TrajectorySequenceSupplier parkLeft = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .strafeTo(new Vector2d(-35, -32))
                .strafeTo(new Vector2d(-55, -32))
                .build();

        TrajectorySequenceSupplier parkCenter = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .strafeTo(new Vector2d(-35, -32))
                .build();

        TrajectorySequenceSupplier parkRight = () -> drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                .strafeTo(new Vector2d(-35, -32))
                .strafeTo(new Vector2d(-12, -32))
                .build();


        Command displayTime = new InstantCommand(() -> System.out.printf("Time Left: %f.2%n", 30 - timer.time()));
        addCommands(
                new PrintCommand("Start Auto"),
                new InstantCommand(() -> {
                    timer.reset();
                    drivetrain.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));
                }),
                new InstantCommand(aprilTagDetector::detect),
                new ParallelCommandGroup(
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
}
