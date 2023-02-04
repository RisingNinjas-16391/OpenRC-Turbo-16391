package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "One Cone Auto Right", group = "Autonomous")
public class CommandAutoOneConeRight extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 1, true, telemetry);
    }
}
