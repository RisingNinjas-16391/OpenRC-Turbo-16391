package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    private final CRServo intake;

    public enum Direction {
        FEED, UNFEED, STOP
    }
    public IntakeSubsystem(@NonNull HardwareMap hwMap) {
        super();
        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(0);

    }
    public void setIntake(Direction direction) {
        switch (direction) {
            case FEED:
                feed();
                break;
            case UNFEED:
                unfeed();
                break;
            case STOP:
                intake.setPower(0);
        }
    }

    public void feed() {
        intake.setPower(-1);
    }

    public void unfeed() {
        intake.setPower(1);
    }
}

