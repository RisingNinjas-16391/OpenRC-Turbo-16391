package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue", group="Autonomous")
//@Disabled
public class BlueAuto extends LinearOpMode {
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

        //TODO: Shoot rings

        //TODO: Wait for rings to launch


        //TODO: turn off everything

        //TODO: turn on climber


        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: Move robot to climb
                .build());

        //TODO: wait for climb
    }
}