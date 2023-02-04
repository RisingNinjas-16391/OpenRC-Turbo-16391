package org.firstinspires.ftc.teamcode.drive.commands.Autos;

import static org.firstinspires.ftc.teamcode.drive.commands.Autos.Left.leftify;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.helpers.TrajectorySequenceSupplier;

public class AutoTrajectories {
    public static TrajectorySequenceSupplier initToStackTrajectory(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.INIT, right))
                .strafeTo(leftify(new Vector2d(35, -24), right))
                .splineToSplineHeading(leftify(Waypoints.STACK, right), (leftify(Math.toRadians(5), right)))
                .build();
    }

    public static TrajectorySequenceSupplier stackToHighTrajectory(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.STACK, right))
                .setTangent(Math.toRadians(0))
                .strafeTo(leftify(new Vector2d(42, -12), right))
                .splineToSplineHeading(leftify(Waypoints.HIGH, right), leftify(Math.toRadians(140), right))
                .setTangent(leftify(Math.toRadians(315), right))
                .build();
    }

    public static TrajectorySequenceSupplier highToStackTrajectory(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.HIGH, right))
                .splineToSplineHeading(leftify(Waypoints.STACK, right), Math.toRadians(0))
                //.setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
                .strafeTo(leftify(new Vector2d(60, -12), right))
                .build();
    }

    public static TrajectorySequenceSupplier highToParkTrajectory(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.HIGH, right))
                .splineToSplineHeading(leftify(Waypoints.PARK_CENTER), leftify(Math.toRadians(270)))
                .build();
    }

    public static TrajectorySequenceSupplier parkLeft(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.PARK_CENTER))
                .strafeTo(leftify(new Vector2d(16, -35)))
                .build();
    }


    public static TrajectorySequenceSupplier parkRight(DrivetrainSubsystem drivetrain, boolean right) {
        return () -> drivetrain.trajectorySequenceBuilder(leftify(Waypoints.PARK_CENTER))
                .strafeTo(leftify(new Vector2d(60, -35)))
                .build();
    }
}
