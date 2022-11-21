package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "Teleop")
public class CommandTeleOP extends CommandOpMode {
    @Override
    public void initialize() {
        RobotContainer robot = new RobotContainer(hardwareMap, false, 0);
    }
}