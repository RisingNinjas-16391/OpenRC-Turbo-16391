package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants;

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
        double driveSpeed;

        if (gamepad1.right_bumper) {
            driveSpeed = 0.4;
        } else if (gamepad1.left_bumper) {
            driveSpeed = 1;
        } else {
            driveSpeed = 0.7;
        }

        if (robot.slide.getCurrentPosition() > 0) {
            double correction = 1 - (robot.slide.getCurrentPosition() / 5000);
            correction /= 2;
            correction += 0.5;
            Log.i("Correction", "Correction: " + correction);
            Log.i("Drive Speed", "Pre: " + driveSpeed + " Post: " + driveSpeed * correction);
            driveSpeed *= correction;
        }

        double forward = -driveSpeed * gamepad1.left_stick_y;
        double strafe = driveSpeed * gamepad1.left_stick_x;
        double turn = - driveSpeed * gamepad1.right_stick_x;

        robot.drivetrain.setWeightedDrivePower(new Pose2d(forward, strafe, turn));

//        robot.slide.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad2.x) {
            robot.slide.setTargetPosition(LiftConstants.bottomPos);
        } else if (gamepad2.dpad_down) {
            robot.slide.setTargetPosition(LiftConstants.feedPos);
        }
        else if (gamepad2.y) {
            robot.slide.setTargetPosition(LiftConstants.lowPos);
        }
        else if (gamepad2.b) {
            robot.slide.setTargetPosition(LiftConstants.midPos);
        }
        else if (gamepad2.a) {
            robot.slide.setTargetPosition(LiftConstants.highPos);
        }
        else if (gamepad2.options){
            while (gamepad2.options) {
                robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slide.setPower(-0.2);
            }
            robot.slide.resetEncoders();
        } else if (gamepad2.back) {
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (robot.slide.getMode() == DcMotor.RunMode.RUN_TO_POSITION && (gamepad2.left_trigger > 0 || gamepad2.right_trigger < 0)) {
            robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (robot.slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.slide.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        }

        robot.intake.setPower(gamepad2.right_bumper ? 1 :gamepad2.left_bumper ? -1 : -0.2);
    }

}
