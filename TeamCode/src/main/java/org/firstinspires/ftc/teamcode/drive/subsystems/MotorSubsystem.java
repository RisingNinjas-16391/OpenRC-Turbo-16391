package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSubsystem {

    public DcMotor Motor;

    public MotorSubsystem(HardwareMap hwMap) {
        Motor = hwMap.get(DcMotorEx.class, "Motor");
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setPower(0);
    }

    public void setPower(double power) {
        Motor.setPower(power);
    }

    public double getPower() {
        return Motor.getPower();
    }

    public int getCurrentPosition() { return Motor.getCurrentPosition(); }

    public void setTargetPosition(int position) { Motor.setTargetPosition(position); }

}
