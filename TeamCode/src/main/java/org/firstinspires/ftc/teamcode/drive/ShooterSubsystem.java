package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private DcMotor motor;

    public ShooterSubsystem(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "shooter");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }
}
