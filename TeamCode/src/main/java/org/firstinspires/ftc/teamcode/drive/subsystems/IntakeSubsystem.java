package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private CRServo intake;

    public IntakeSubsystem(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addLine("Intake Power")
                .addData("intake", intake.getPower());
    }

    public void feed() {
        intake.setPower(-1);
    }

    public void unfeed() {
        intake.setPower(1);
    }
}

