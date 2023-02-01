package org.firstinspires.ftc.teamcode.drive.opmode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.RobotContainer;

@Autonomous(name = "Multi Cone Auto Left", group = "Autonomous")
public class CommandAutoMultiConeLeft extends CommandOpMode {
    @Override
    public void initialize() {
        new RobotContainer(hardwareMap, 3, telemetry);
    }
}
