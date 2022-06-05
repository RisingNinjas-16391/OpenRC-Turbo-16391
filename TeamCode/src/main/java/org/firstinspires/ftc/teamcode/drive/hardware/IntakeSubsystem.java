package org.firstinspires.ftc.teamcode.drive.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public DcMotor motor;
    public IntakeSubsystem() {

    }
    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
}
