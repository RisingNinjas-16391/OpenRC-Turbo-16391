package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red", group="Autonomous")
//@Disabled
public class RedAuto extends LinearOpMode {
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

        // Spin up flywheel
        robot.flywheel.setPower(1);
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: Move robot to score
                .build());

        // Shoot rings
        robot.kicker.setPower(1);
        // Wait for rings to launch
        sleep(2000);

        // turn off everything
        robot.flywheel.setPower(0);
        robot.kicker.setPower(0);

        // turn on climber
        robot.climber.setPower(1);

        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: Move robot to climb
                .build());

        // wait for climb
        sleep(1000);
    }
}