package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "Autonomous good", group = "Autonomous")
public class CommandAuto2 extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 2, telemetry);
    }
}
