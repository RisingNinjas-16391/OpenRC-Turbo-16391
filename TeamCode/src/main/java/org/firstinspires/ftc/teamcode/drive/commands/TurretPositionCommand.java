package org.firstinspires.ftc.teamcode.drive.commands;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.minHeightTurret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretPositionCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TurretSubsystem turret;
    private final DoubleSupplier liftHeight;
    private boolean toggled = false;
    private final boolean position;

    public TurretPositionCommand(final TurretSubsystem turret, final DoubleSupplier liftHeight, final boolean position) {
        this.turret = turret;
        this.liftHeight = liftHeight;
        this.position = position;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        this.toggled = false;
    }

    @Override
    public void execute() {
        if (liftHeight.getAsDouble() > minHeightTurret && !toggled) {
            turret.togglePosition(position);
            toggled = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return !turret.isBusy() && toggled;
    }
}
