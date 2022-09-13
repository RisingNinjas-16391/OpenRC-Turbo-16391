package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;

@TeleOp(name = "TestOp", group = "Teleop")
public class TestOP extends LinearOpMode {

    Hardware robot = new Hardware();   //Uses heavily modified untested hardware
    boolean pressed = false;
    boolean d = false;
    int index = 0;


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


        if (gamepad1.x) {
            if (d) {
                if (index <= 0) {
                    index = 7;
                } else {
                    index --;
                }
            }
            d = false;
            pressed = true;
        } else if (gamepad1.b) {
            if (d) {
                if (index >= 7) {
                    index = 0;
                } else {
                    index ++;
                }
            }
            d = false;
            pressed = true;
        }
        else {
            d = true;
        }

        if (pressed) {
            robot.drivetrainSubsystem.setMotorPowers(0, 0, 0, 0);
            robot.arm.setPower(0);
            robot.intake.setPower(0);
            // robot.carousel.setPower(0);
            pressed = false;
        }

        switch (index) {
            case 0: {
                robot.drivetrainSubsystem.setMotorPowers(0, 0, 0, 0);
                robot.arm.setPower(0);
                robot.intake.setPower(0);
                // robot.carousel.setPower(0);
                break;
            }
            case 1: {
                robot.drivetrainSubsystem.setMotorPowers(forward, 0, 0, 0);
                break;
            }
            case 2: {
                robot.drivetrainSubsystem.setMotorPowers(0, forward, 0, 0);
                break;
            }
            case 3: {
                robot.drivetrainSubsystem.setMotorPowers(0, 0, forward, 0);
                break;
            }
            case 4: {
                robot.drivetrainSubsystem.setMotorPowers(0, 0, 0, forward);
                break;
            }
            case 5: {
                robot.arm.setPower(forward);
                break;
            }
            case 6: {
                robot.intake.LeftIntake.setPower(forward);
                break;
            }
            case 7: {
                robot.intake.RightIntake.setPower(forward);
                break;
            }
//            case 8: {
//                robot.carousel.setPower(forward);
//            }
        }
        telemetry.addData("Index", index);
        telemetry.addLine("joystick stats");
        telemetry.addData("power: ", forward);
    }
}
