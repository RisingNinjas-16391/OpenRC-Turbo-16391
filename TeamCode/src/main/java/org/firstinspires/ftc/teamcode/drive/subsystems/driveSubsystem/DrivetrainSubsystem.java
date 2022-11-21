package org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class DrivetrainSubsystem extends SubsystemBase {

    private MecanumDrive drivetrain;
    private Telemetry telemetry;

    private double speedMultiplier = 1;

    public DrivetrainSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.drivetrain = new MecanumDrive(hwMap);
        this.telemetry = telemetry;
    }

    public Pose2d getPoseEstimate() {
        return drivetrain.getPoseEstimate();
    }

    public void drive(double forward, double strafe, double turn) {
        drivetrain.setWeightedDrivePower(new Pose2d(forward * speedMultiplier, strafe * speedMultiplier, turn * speedMultiplier));
    }

    public void runTrajectory(TrajectorySequence trajectory) {
        drivetrain.followTrajectorySequenceAsync(trajectory);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drivetrain.trajectorySequenceBuilder(startPose);
    }

    public void setPoseEstimate(Pose2d pose) {
        drivetrain.setPoseEstimate(pose);
    }

    @Override
    public void periodic() {
        telemetry.addLine("Drive Encoder ticks")
                .addData("Front Left", drivetrain.getWheelPositions().get(0))
                .addData("Front Right", drivetrain.getWheelPositions().get(3))
                .addData("Back Left", drivetrain.getWheelPositions().get(1))
                .addData("Back Right", drivetrain.getWheelPositions().get(2));
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }
}
