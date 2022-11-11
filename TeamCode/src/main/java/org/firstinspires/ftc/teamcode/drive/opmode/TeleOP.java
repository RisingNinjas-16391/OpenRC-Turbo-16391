package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        double driveSpeed = 1;

        if (gamepad1.right_bumper) {
            driveSpeed = 0.5;
        }
        double forward = -driveSpeed * gamepad1.left_stick_y * 0.7;
        double strafe = driveSpeed * gamepad1.left_stick_x * 0.7;
        double turn = driveSpeed * gamepad1.right_stick_x* 0.7;


        double[] driveValues = {
                forward - strafe + turn,
                forward + strafe + turn,
                forward - strafe - turn,
                forward + strafe - turn
        };

        robot.drivetrain.setMotorPowers(driveValues[0], driveValues[1], driveValues[2], driveValues[3]);

//        robot.slide.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad2.x) {
            robot.slide.setTargetPosition(100);
        }
        else if (gamepad2.y) {
            robot.slide.setTargetPosition(800);
        }
        else if (gamepad2.b) {
            robot.slide.setTargetPosition(3500);
        }
        else if (gamepad2.a) {
            robot.slide.setTargetPosition(4000);
        }
        else if (gamepad2.options){
            while (gamepad2.options) {
                robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slide.setPower(-0.2);
            }
            robot.slide.resetEncoders();
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        // robot.intake.setPower(gamepad2.right_bumper ? -1 :gamepad2.left_bumper ? 1 : -0.1);
        if(gamepad2.left_bumper) {
            robot.intake.setPower(-1);
            robot.slide.setTargetPosition((int) Math.abs(robot.slide.getCurrentPosition() - 100));
        }
        if(gamepad2.right_bumper) {
            robot.intake.setPower(1);
        } else {
            robot.intake.setPower(-0.1);
        }


    }

}
