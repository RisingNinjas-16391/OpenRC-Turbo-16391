package org.firstinspires.ftc.teamcode.drive.opmode;

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
        while (!isStopRequested() && opModeIsActive()) {
            driveInput();
            slideInput();
            robot.intake.setPower(gamepad2.left_bumper ? 1 : -1);       // Intake control (always intaking)
            robot.displayTelemetry(telemetry);
            robot.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }

    // Turret Method
    private void turretControl() {
    }

    // Slide input control
    private void slideInput() {
        if (gamepad2.x) {   // Low preset
            robot.slide.setTargetPosition(LiftConstants.BOTTOM_POS);
        } else if (gamepad2.dpad_down) {    // Intake preset
            robot.slide.setTargetPosition(LiftConstants.FEED_POS);
        } else if (gamepad2.y) {    // Low scoring preset
            robot.slide.setTargetPosition(LiftConstants.LOW_POS);
        } else if (gamepad2.b) {    // Middle scoring preset
            robot.slide.setTargetPosition(LiftConstants.MID_POS);
        } else if (gamepad2.a) {    // High scoring preset
            robot.slide.setTargetPosition(LiftConstants.HIGH_POS);
        } else if (gamepad2.options){     // Reset encoder
            robot.slide.resetEncoders();
        } else if (gamepad2.back) {     // Turn back PID
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (robot.slide.getMode() == DcMotor.RunMode.RUN_TO_POSITION
                && (gamepad2.left_trigger > 0 || gamepad2.right_trigger < 0)) { // Turn off pid when detect manual override
            robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (robot.slide.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {  // Manual override active when pids off
            robot.slide.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        }

    }

    // Drive input control
    private void driveInput() {
        double driveSpeed;

        // Slowmode control
        if (gamepad1.right_bumper) {
            driveSpeed = 0.4;               // Slowmode
        } else if (gamepad1.left_bumper) {
            driveSpeed = 1;                 // Fastmode
        } else {
            driveSpeed = 0.7;               // Regular speed
        }

        // Slow down robot max speed when slides are extended
        if (robot.slide.getCurrentPosition() > 0) {
            double correction = 1 - (robot.slide.getCurrentPosition() / 5000);
            correction /= 2;
            correction += 0.5;
            driveSpeed *= correction;
        }


        // Gamepad inputs multiplied by the drive speed multiplier
        double forward = - driveSpeed * gamepad1.left_stick_y;
        double strafe = driveSpeed * gamepad1.left_stick_x;
        double turn = - driveSpeed * gamepad1.right_stick_x;

        // Actual drive
        robot.drivetrain.setWeightedDrivePower(new Pose2d(forward, strafe, turn));
    }
}
