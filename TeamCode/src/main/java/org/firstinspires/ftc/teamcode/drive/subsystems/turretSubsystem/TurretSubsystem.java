package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class TurretSubsystem {

    public DcMotorEx turret;

    public TurretSubsystem(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        // TODO: Turn back to brake mode
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0, 0, 0, 0));
    }

    public int getCurrentPosition() { return turret.getCurrentPosition(); }

    public void setTargetPosition(int position) { turret.setTargetPosition(position); }

}
