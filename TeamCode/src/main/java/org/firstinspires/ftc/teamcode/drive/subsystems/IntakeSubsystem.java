package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public DcMotor LeftIntake;
    public DcMotor RightIntake;

    public IntakeSubsystem(HardwareMap hwMap) {
        LeftIntake = hwMap.get(DcMotorEx.class, "LeftIntake");
        LeftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftIntake.setPower(0);

        RightIntake = hwMap.get(DcMotorEx.class, "RightIntake");
        RightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        RightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightIntake.setPower(0);
    }

    public void setPower(double power) {
        LeftIntake.setPower(power);
        RightIntake.setPower(power);
    }

}
