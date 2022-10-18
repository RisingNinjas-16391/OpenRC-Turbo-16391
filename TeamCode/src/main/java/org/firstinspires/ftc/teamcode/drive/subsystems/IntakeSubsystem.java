package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    public DcMotor Intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        Intake = hwMap.get(DcMotorEx.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setPower(0);
    }

    public void setPower(double power) {
        Intake.setPower(power);
    }

    public double getPower() {
        return Intake.getPower();
    }

    public int getCurrentPosition() { return Intake.getCurrentPosition(); }

    public void setTargetPosition(int position) { Intake.setTargetPosition(position); }

}
