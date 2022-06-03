package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
    private DcMotor motor;
    public static final int TOP_TICK = -1500;
    public static final int BOTTOM_TICK = 0;
    public static final int DEADZONE = 20;

    public LiftSubsystem(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "lift");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
        if (getCurrentPosition() < TOP_TICK) {
            motor.setPower(0.005 * (TOP_TICK + DEADZONE - getCurrentPosition()));
        }
        else if (getCurrentPosition() > BOTTOM_TICK) {
            motor.setPower(0.01 * (BOTTOM_TICK - DEADZONE - getCurrentPosition()));
        } else if ((power > 0 && motor.getCurrentPosition() > BOTTOM_TICK - DEADZONE) || (power < 0 && motor.getCurrentPosition() < TOP_TICK + DEADZONE)) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
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
