package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {



    private DcMotorEx motor;
    private PIDFController controller;
    private DcMotor.RunMode mode;


    public LiftSubsystem(HardwareMap hwMap) {
        controller = new PIDFController(LiftConstants.kPosPID, LiftConstants.kV, LiftConstants.kA, LiftConstants.kS, (x, v) -> LiftConstants.kG);
        controller.setInputBounds(0, LiftConstants.maxHeight);
        controller.setOutputBounds(-1, 1);
        mode = DcMotor.RunMode.RUN_TO_POSITION;

        motor = hwMap.get(DcMotorEx.class, LiftConstants.name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(int targetPos) {
        controller.setTargetPosition(targetPos);
    }

    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
    }

    public double getPower() {
        return motor.getPower();
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setTargetPosition(int position) { controller.setTargetPosition(position); }

    public void update() {
        switch (mode) {
            case RUN_TO_POSITION:
                motor.setPower(controller.update(motor.getCurrentPosition(), motor.getVelocity()));
                break;
            case RUN_USING_ENCODER:
                motor.setPower(0);
                break;
        }
        // Soft limit
        if (motor.getCurrentPosition() + LiftConstants.endMargin > LiftConstants.maxHeight && motor.getPower() > 0) {
            motor.setPower(-0.1);
        } else if (motor.getCurrentPosition() - LiftConstants.endMargin > 0 && motor.getPower() < 0) {
            motor.setPower(0.1);
        }
    }

}

