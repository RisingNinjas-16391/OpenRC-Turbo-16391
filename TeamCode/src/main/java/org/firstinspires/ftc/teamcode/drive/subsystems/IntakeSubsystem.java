package org.firstinspires.ftc.teamcode.drive.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public CRServo intake;

    public IntakeSubsystem(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(0);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public double getPower() {
        return intake.getPower();
    }

}

