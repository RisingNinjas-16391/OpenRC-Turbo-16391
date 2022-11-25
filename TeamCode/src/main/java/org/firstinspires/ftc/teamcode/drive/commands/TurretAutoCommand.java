package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.minHeightTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretAutoCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TurretSubsystem turret;
    private final DoubleSupplier liftHeight;
    private final BooleanSupplier mode;

    public TurretAutoCommand(TurretSubsystem turret, DoubleSupplier liftHeight, BooleanSupplier mode) {
        this.turret = turret;
        this.liftHeight = liftHeight;
        this.mode = mode;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (liftHeight.getAsDouble() > minHeightTurret) {
            turret.togglePosition(mode.getAsBoolean());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
