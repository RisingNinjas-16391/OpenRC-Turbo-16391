package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "Autonomous", group = "Autonomous")
public class CommandAuto extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 0, telemetry);
    }
}
