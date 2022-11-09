package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem {

    public DcMotor turret;

    public TurretSubsystem(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0);
    }

    public int getCurrentPosition() { return turret.getCurrentPosition(); }

    public void setTargetPosition(int position) { turret.setTargetPosition(position); }

}
