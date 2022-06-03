package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KickerSubsystem {
    private DcMotor motor;

    public KickerSubsystem(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "kicker");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
            motor.setPower(power);
    }
}
