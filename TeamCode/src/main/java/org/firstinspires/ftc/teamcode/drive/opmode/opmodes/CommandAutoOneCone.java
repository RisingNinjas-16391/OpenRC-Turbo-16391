package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "One Cone Auto", group = "Autonomous")
public class CommandAutoOneCone extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 1, telemetry);
    }
}
