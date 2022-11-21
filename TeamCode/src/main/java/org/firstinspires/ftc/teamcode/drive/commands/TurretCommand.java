package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.minHeightTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TurretSubsystem turret;
    private final DoubleSupplier liftHeight;

    public TurretCommand(TurretSubsystem turret, DoubleSupplier liftHeight) {
        this.turret = turret;
        this.liftHeight = liftHeight;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (liftHeight.getAsDouble() > minHeightTurret) {
            turret.togglePosition();
        }
    }
}
