package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TurretSubsystem turret;
    private BooleanSupplier toggle;

    public TurretCommand(TurretSubsystem turret, BooleanSupplier toggle) {
        this.turret = turret;
        this.toggle = toggle;
    }

    @Override
    public void execute() {
        turret.togglePosition(toggle.getAsBoolean());
    }
}
