package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous", group="Autonomous")
//@Disabled

public class Auto extends AutonomousTemplate {
    Hardware robot = new Hardware();   //Uses heavily modified untested hardware

    public void initialize() {
        robot.init(hardwareMap);
    }

    public void parkLeft() {
        //TODO: Trajectories for LEFT auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-58, -12, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-58, -34,  Math.toRadians(90)))
                        .build()
        );
    }

    public void parkCenter() {
        //TODO: Trajectories for CENTER auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-58, -12, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                        .build()
        );
    }

    public void parkRight() {
        //TODO: Trajectories for RIGHT auto
        robot.drivetrain.followTrajectorySequence(
                robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-58, -12, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-15, -34,  Math.toRadians(90)))
                        .build()
        );
    }

    public void regularAutonomous() {
        //TODO: ADD AUTO
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            TrajectorySequence initToStack = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58, -60,  Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(58, -12,  Math.toRadians(180)), Math.toRadians(90))
                .build();

            TrajectorySequence stackToHigh = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-58, -12, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(38, -12,  Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(32, -7,  Math.toRadians(45)))
                    .build();

            TrajectorySequence highToStack = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(-32, -7, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(38, -12,  Math.toRadians(180)))
                    .splineToLinearHeading(new Pose2d(58, -12,  Math.toRadians(180)), Math.toRadians(180))
                    .build();

            robot.drivetrain.followTrajectorySequence(initToStack);

            robot.slide.setTargetPosition(55);
            robot.intake.setPower(1);
            robot.slide.setTargetPosition(50);
            robot.intake.setPower(0);


            robot.drivetrain.followTrajectorySequence(stackToHigh);
            robot.slide.setTargetPosition(300);
            robot.intake.setPower(-1);
            robot.intake.setPower(0);


            robot.drivetrain.followTrajectorySequence(highToStack);
            robot.slide.setTargetPosition(50);
            robot.intake.setPower(1);
            robot.slide.setTargetPosition(45);
            robot.intake.setPower(0);


            robot.drivetrain.followTrajectorySequence(stackToHigh);
            robot.slide.setTargetPosition(300);
            robot.intake.setPower(-1);
            robot.intake.setPower(0);


            robot.drivetrain.followTrajectorySequence(highToStack);
            robot.slide.setTargetPosition(45);
            robot.intake.setPower(1);
            robot.slide.setTargetPosition(40);
            robot.intake.setPower(0);
    }
}