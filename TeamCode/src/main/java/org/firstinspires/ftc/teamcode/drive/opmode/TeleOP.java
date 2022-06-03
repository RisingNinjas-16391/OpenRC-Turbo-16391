package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Hardware;

@TeleOp(name = "TeleOp", group = "Teleop")
public class TeleOP extends LinearOpMode {

    Hardware robot = new Hardware();   //Uses heavily modified untested hardware

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
        double forward  = -gamepad1.left_stick_y;
        double strafe   = -gamepad1.left_stick_x;
        double turn     = -gamepad1.right_stick_x;
        double[] driveValues = {
                forward - strafe - turn,
                forward + strafe - turn,
                forward - strafe + turn,
                forward + strafe + turn
        };

        robot.drivetrainSubsystem.setMotorPowers(
                driveValues[0],
                driveValues[1],
                driveValues[2],
                driveValues[3]
        );
        robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        // YOU CAN TOUCH THIS
        if (gamepad1.left_bumper) {
            robot.kicker.setPower(-0.5);      //  Spin up
        } else if (gamepad1.right_bumper) {
            robot.kicker.setPower(0.5);
        } else {
            robot.kicker.setPower(0);
        }
        if (gamepad1.dpad_down) {
            robot.flywheel.setPower(0);
        } else if (gamepad1.dpad_left) {
            robot.flywheel.setPower(.5);
        } else if (gamepad1.dpad_up) {
            robot.flywheel.setPower(.7);
        } else if (gamepad1.dpad_right) {
            robot.flywheel.setPower(1);
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