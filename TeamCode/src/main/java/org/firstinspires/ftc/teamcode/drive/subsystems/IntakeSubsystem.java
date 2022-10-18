package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    public Servo Intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        Intake = hwMap.get(Servo.class, "Intake");
    }

    public void setPosition(double position) {
        Intake.setPosition(position);
    }

}
