package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Autonomous", group="Autonomous")
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

    //TODO: Finish auto!
    public void editHere() {

        // Lift arm to scoring height
        robot.lift.setPower(0.5);
        sleep(1000);
        robot.lift.setPower(0);

        // Drive to score preload ring
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: Drive to score
                .build());

        // Eject ring
        robot.intake.setPower(1);
        sleep(1000);
        robot.intake.setPower(0);

        // Turn on climber
        robot.climber.setPower(1);
        // Drive to climbing spot
        robot.drivetrainSubsystem.followTrajectorySequence(robot.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
                // TODO: Drive to climb
                .build());
        // wait to climb a bit
        sleep(2000);

    }
}