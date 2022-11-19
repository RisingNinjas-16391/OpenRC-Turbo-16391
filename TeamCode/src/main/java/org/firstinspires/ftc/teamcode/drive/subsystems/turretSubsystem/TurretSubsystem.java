package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.homePos;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.kPosPID;
import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.otherSidePos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem {

    public DcMotorEx turret;
    private boolean toggle = false;

    public TurretSubsystem(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        // TODO: Turn back to brake mode
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, kPosPID);
    }

    public int getCurrentPosition() { return turret.getCurrentPosition(); }

    public void setTargetPosition(int position) { turret.setTargetPosition(position); }

    public void togglePosition() {
        toggle = !toggle;
        if (toggle){
            turret.setTargetPosition(homePos);
        } else {
            turret.setTargetPosition(otherSidePos);
        }
    }

    public void togglePosition(boolean toggle) {
        this.toggle = toggle;
        if (this.toggle){
            turret.setTargetPosition(homePos);
        } else {
            turret.setTargetPosition(otherSidePos);
        }
    }

}
