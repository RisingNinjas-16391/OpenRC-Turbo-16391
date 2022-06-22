package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Auto", group="Autonomous")
//@Disabled
public class RedAutonomous extends LinearOpMode {
    /* Declare OpMode members. */
    static Hardware robot = new Hardware();
    static ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        telemetry.addLine("Status: Booting");
        telemetry.update();

        robot.init(hardwareMap);
        telemetry.addData("Robot initialized: ", true);
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Waiting for start");
        telemetry.update();
        telemetry.setAutoClear(true);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        editHere();

    }


    public void editHere() {
        //TODO: Add auto
        sleep(1000);

        robot.drivetrainSubsystem.followTrajectorySequence(
                robot.drivetrainSubsystem.trajectorySequenceBuilder(
                        robot.drivetrainSubsystem.getPoseEstimate())
                            // Drive to shooting position
                            .forward(120)
                            .build());

        // Shooting balls
        robot.shooter.setPower(1);
        robot.agitator.setPower(1);
        sleep(1000);

        // Turn off shooter
        robot.shooter.setPower(0);
        robot.agitator.setPower(0);

        // Climber on
        robot.climber.setPower(-1);

        robot.drivetrainSubsystem.followTrajectorySequence(
                robot.drivetrainSubsystem.trajectorySequenceBuilder(
                        robot.drivetrainSubsystem.getPoseEstimate())
                            // drive to climb
                            .strafeRight(40)
                            .back(47)
                            .build());

        sleep(2000);    // Time to climb
        // Turn off climber
        robot.climber.setPower(0);
    }
}