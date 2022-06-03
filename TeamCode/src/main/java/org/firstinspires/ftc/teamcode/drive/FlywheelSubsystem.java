package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelSubsystem {
    private DcMotor motor;

    public FlywheelSubsystem(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
            motor.setPower(power);
    }
}
