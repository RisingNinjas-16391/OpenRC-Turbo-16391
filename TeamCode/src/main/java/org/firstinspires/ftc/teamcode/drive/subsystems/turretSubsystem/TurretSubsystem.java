package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.homePos;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.kPosPID;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.name;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.otherSidePos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    public DcMotorEx turret;
    private Telemetry telemetry;
    private boolean toggle;

    public TurretSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        turret = hwMap.get(DcMotorEx.class, name);
        // TODO: Turn back to brake mode
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, kPosPID);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addLine("Turret Encoder Position")
                .addData("Ticks: ", turret.getCurrentPosition());
    }

    public void togglePosition(boolean toggle) {
        this.toggle = toggle;
        if (toggle) {
            turret.setTargetPosition(homePos);
        } else {
            turret.setTargetPosition(otherSidePos);
        }
    }

    public void togglePosition() {
        toggle = !toggle;
        togglePosition(toggle);
    }

}
