package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.minHeightTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TurretSubsystem turret;
    private final LiftSubsystem lift;
    private boolean toggle;

    public TurretCommand(TurretSubsystem turret, LiftSubsystem lift) {
        this.turret = turret;
        this.lift = lift;
        toggle = true;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (lift.getCurrentHeight() > minHeightTurret) {
            turret.togglePosition(toggle);
        }

        toggle = !toggle;
    }
}
