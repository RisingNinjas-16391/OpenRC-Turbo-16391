package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    public CRServo Intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        Intake = hwMap.get(CRServo.class, "Intake");
        Intake.setDirection(CRServo.Direction.FORWARD);
        Intake.setPower(0);
    }

    public void setPower(int power) {
        Intake.setPower(power);
    }

    public double getPower() {
        return Intake.getPower();
    }

}
