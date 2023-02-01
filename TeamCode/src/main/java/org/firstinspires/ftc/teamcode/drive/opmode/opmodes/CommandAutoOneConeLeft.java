package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "One Cone Auto Left", group = "Autonomous")
public class CommandAutoOneConeLeft extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 1, telemetry);
    }
}
