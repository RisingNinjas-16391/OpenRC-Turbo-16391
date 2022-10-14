package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous")
//@Disabled
public class Auto extends LinearOpMode {
    /* Declare OpMode members. */
    static Hardware hardware = new Hardware();
    static ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        telemetry.addLine("Status: Booting");
        telemetry.update();

        hardware.init(hardwareMap);
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
        //TODO: ADD AUTO
//        hardware.drivetrainSubsystem.followTrajectorySequence(hardware.drivetrainSubsystem.trajectorySequenceBuilder(new Pose2d())
//                // TODO: ADD ROBOT CODE BELOW!
//                .forward(40)
//                .back(40)
//                .strafeLeft(40)
//                .strafeRight(40)
//                .turn(Math.toRadians(90))
//                .build());
    }
}