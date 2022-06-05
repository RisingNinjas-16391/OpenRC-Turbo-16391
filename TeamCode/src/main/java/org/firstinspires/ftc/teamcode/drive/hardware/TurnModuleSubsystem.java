package org.firstinspires.ftc.teamcode.drive.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

/* Subsystem for the turn modules on a swerve module */
public class TurnModuleSubsystem {
    private CRServo turnServo;
    private Encoder turnEncoder;

    public void init(HardwareMap hwMap, String turnName, String encoderName) {
        turnServo = hwMap.get(CRServo.class, turnName);
        turnEncoder = hwMap.get(Encoder.class, encoderName);
    }

    public void setModuleOrientation(double targetAngle) {
        double currentAngle = getModuleOrientation();
        double error = closestAngle(targetAngle, currentAngle);
        if (error > 10) {
            turnServo.setPower(1);
        } else if (error < -10) {
            turnServo.setPower(-1);
        } else {
            turnServo.setPower(0);
        }

    }


    private double closestAngle(double target, double current) {
        double error = Math.toDegrees(target)-Math.toDegrees(current);
        if (Math.abs(error) > 180) {
            error = 360 - error;
        }
        return error;
    }

    public double getModuleOrientation() {
        return Math.toRadians(turnEncoder.getCurrentPosition()*0.087891);

    }
}
