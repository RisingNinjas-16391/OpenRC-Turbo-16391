package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@RequiresApi(api = Build.VERSION_CODES.N)
@Autonomous(name="Just Park", group="Autonomous")
//@Disabled

public class AutoSimple extends AutonomousTemplate {
    Hardware robot = new Hardware(this::opModeIsActive, this::isStopRequested);   //Uses heavily modified untested hardware

    public void initialize() {
        Log.i("Auto", "Init hardware");
        robot.init(hardwareMap);
        robot.drivetrain.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));
    }

    public void parkLeft() {
        //TODO: Trajectories for LEFT auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-10, -34,  Math.toRadians(90)))
                        .build()
        );
    }

    public void parkCenter() {
        //TODO: Trajectories for CENTER auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                        .build()
        );
    }

    public void parkRight() {
        //TODO: Trajectories for RIGHT auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-33, -34, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60, -34,  Math.toRadians(90)))
                        .build()
        );
    }


    public void regularAutonomous() {}
}