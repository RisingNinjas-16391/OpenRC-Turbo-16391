package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedAuto", group="Autonomous")
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
        //TODO: Drive to Block Depot
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: ADD ROBOT CODE BELOW!

                .build());

        //TODO: Drop off Block
        robot.intake.setPower(-1);
        sleep(500);
        robot.intake.setPower(0);

        //TODO: Drive to pick up block
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: ADD ROBOT CODE BELOW!

                .build());

        robot.intake.setPower(-1);
        sleep(500);
        robot.intake.setPower(0);

        //TODO: Drive to drop off block
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: ADD ROBOT CODE BELOW!

                .build());

        //TODO: Drop off Block
        robot.intake.setPower(-1);
        sleep(500);
        robot.intake.setPower(0);

        //TODO: Drive to climb
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: ADD ROBOT CODE BELOW!

                .build());

        robot.climber.setPower(0);

    }
}