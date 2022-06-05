package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.hardware.Robot;

@TeleOp(name = "Reset Lift", group = "Teleop")
public class ResetPosition extends LinearOpMode {

    Robot robot = new Robot();   //Uses heavily modified untested hardware

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Press Start to Begin");    //
        telemetry.update();
        waitForStart();
        robot.lift.reset();
        while (opModeIsActive()) {
            robot.displayTelemetry(telemetry);
            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }
}