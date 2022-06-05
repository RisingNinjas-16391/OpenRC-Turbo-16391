package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.hardware.Robot;

@TeleOp(name = "TeleOp", group = "Teleop")
public class TeleOP extends LinearOpMode {

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
        while (opModeIsActive()) {
            editHere();
            robot.displayTelemetry(telemetry);
            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }

    public void editHere() {
        robot.drivetrainSubsystem.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        // YOU CAN TOUCH THIS
        if (gamepad1.left_bumper) {
            robot.lift.setPower(-0.5);      //  Upward power
        } else if (gamepad1.right_bumper) {
            robot.lift.setPower(0.5);       //  Downward power
        } else {
            robot.lift.setPower(0);
        }
        if (gamepad1.a) {
            robot.climber.setPower(-1);
        } else if (gamepad1.x) {
            robot.climber.setPower(1);
        } else {
            robot.climber.setPower(0);
        }
    }
}