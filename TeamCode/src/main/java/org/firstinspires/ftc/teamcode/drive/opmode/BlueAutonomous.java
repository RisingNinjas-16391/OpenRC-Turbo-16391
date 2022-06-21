package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Auto", group="Autonomous")
//@Disabled
public class BlueAutonomous extends LinearOpMode {
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
        sleep(1000);    // wait for balls to be dropped

        robot.drivetrainSubsystem.followTrajectorySequence(
                robot.drivetrainSubsystem.trajectorySequenceBuilder(
                                robot.drivetrainSubsystem.getPoseEstimate())
                        // TODO: Drive to shooting position

                        .build());

        // TODO: Shooting balls

        // TODO: Turn off shooter

        // TODO: Climber On
        robot.climber.setPower(1);

        robot.drivetrainSubsystem.followTrajectorySequence(
                robot.drivetrainSubsystem.trajectorySequenceBuilder(
                                robot.drivetrainSubsystem.getPoseEstimate())
                        //TODO: drive to climb

                        .build());

        // TODO: Time to climb

        // TODO: Turn off climber

    }
}