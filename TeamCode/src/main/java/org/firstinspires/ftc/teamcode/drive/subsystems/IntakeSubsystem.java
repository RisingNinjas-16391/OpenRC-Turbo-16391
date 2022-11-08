package org.firstinspires.ftc.teamcode.drive.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public CRServo motor;

    public IntakeSubsystem(@NonNull HardwareMap hwMap) {
        motor = hwMap.get(CRServo.class, "intake");
        motor.setDirection(CRServo.Direction.FORWARD);
        motor.setPower(0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

}

