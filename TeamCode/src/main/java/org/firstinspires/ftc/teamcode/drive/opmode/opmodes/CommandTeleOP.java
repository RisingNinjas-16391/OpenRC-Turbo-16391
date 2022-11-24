package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@TeleOp(name = "TeleOp", group = "Teleop")
public class CommandTeleOP extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, gamepad1, gamepad2, telemetry);

    }
}