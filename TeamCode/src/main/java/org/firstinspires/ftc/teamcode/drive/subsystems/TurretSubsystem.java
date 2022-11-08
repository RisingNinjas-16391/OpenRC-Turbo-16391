package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem {

    public DcMotor Turret;

    public TurretSubsystem(HardwareMap hwMap){
        Turret = hwMap.get(DcMotorEx.class, "turret");
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setDirection(DcMotorSimple.Direction.FORWARD);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(0);
    }

    public int getCurrentPosition() { return Turret.getCurrentPosition(); }

    public void setTargetPosition(int position) { Turret.setTargetPosition(position); }

}
