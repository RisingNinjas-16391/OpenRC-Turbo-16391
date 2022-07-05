package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
    private DcMotor motor;

    public LiftSubsystem(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "lift");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
        motor.setPower(power * 0.7);
    }
    public void reset() {
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
