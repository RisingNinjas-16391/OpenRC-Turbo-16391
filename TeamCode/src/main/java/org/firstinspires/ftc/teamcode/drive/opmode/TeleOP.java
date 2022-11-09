package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;

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
            robot.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }

    public void editHere() {
        // TODO: ADD TELEOP CODE
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double[] driveValues = {
                forward - strafe + turn,
                forward + strafe + turn,
                forward - strafe - turn,
                forward + strafe - turn
        };

        robot.drivetrain.setMotorPowers(driveValues[0], driveValues[1], driveValues[2], driveValues[3]);

        robot.slide.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.x) {
            robot.slide.setTargetPosition(100);
        }
        else if (gamepad1.y) {
            robot.slide.setTargetPosition(3200);
        }
        else if (gamepad1.b) {
            robot.slide.setTargetPosition(4000);
        }

        robot.intake.setPower(gamepad1.right_bumper ? 1 :gamepad1.left_bumper ? -1 : 0);


    }

}
