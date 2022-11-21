package org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DrivetrainSubsystem extends SubsystemBase {

    private MecanumDrive drivetrain;

    public DrivetrainSubsystem(HardwareMap hwMap) {
        MecanumDrive drivetrain = new MecanumDrive(hwMap);
    }

    public Pose2d getPose() {
        return drivetrain.getPoseEstimate();
    }

    public void drive(double forward, double strafe, double turn) {
        drivetrain.setWeightedDrivePower(new Pose2d(forward, strafe, turn));
    }

    public void runTrajectory(TrajectorySequence trajectory) {
        drivetrain.followTrajectorySequence(trajectory);
    }

    @Override
    public void periodic() {
        telemetry.addLine("Drive Encoder ticks")
                .addData("Front Left", drivetrain.getWheelPositions().get(0))
                .addData("Front Right", drivetrain.getWheelPositions().get(3))
                .addData("Back Left", drivetrain.getWheelPositions().get(1))
                .addData("Back Right", drivetrain.getWheelPositions().get(2));
    }
}
