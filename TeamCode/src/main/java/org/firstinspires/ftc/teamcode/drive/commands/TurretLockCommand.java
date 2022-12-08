package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;


public class TurretLockCommand extends CommandBase {
    private final TurretSubsystem turret;

    public TurretLockCommand(TurretSubsystem turret) {
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        //turret.setPosition();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
