package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClimberSubsystem {
    public DcMotor climber;

    public ClimberSubsystem(HardwareMap hwMap) {
        climber = hwMap.get(DcMotorEx.class, "climber");
        climber.setDirection(DcMotorSimple.Direction.FORWARD);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setDirection(DcMotorSimple.Direction.FORWARD);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climber.setPower(0);
    }

    public void setPower(double power) {
        climber.setPower(power);
    }
}
