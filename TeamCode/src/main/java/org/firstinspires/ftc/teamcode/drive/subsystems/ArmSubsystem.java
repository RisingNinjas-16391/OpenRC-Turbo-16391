package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {

    public DcMotor ArmMotor;

    public ArmSubsystem(HardwareMap hwMap) {
        ArmMotor = hwMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);
    }

    public void setPower(double power) {
        ArmMotor.setPower(power);
    }

    public double getPower() {
        return ArmMotor.getPower();
    }

    public int getCurrentPosition(){
        return ArmMotor.getCurrentPosition();
    }


}

