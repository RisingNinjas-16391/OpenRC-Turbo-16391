package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Subsystem representing lifting mechanism*/
public class LiftSubsystem {
    private DcMotorEx motor;
    private PIDFController controller;
    private DcMotor.RunMode mode;


    public LiftSubsystem(@NonNull HardwareMap hwMap) {
        controller = new PIDFController(LiftConstants.kPosPID, LiftConstants.kV, LiftConstants.kA, LiftConstants.kS, (x, y) -> LiftConstants.kG);
//        controller.setInputBounds(0, LiftConstants.maxHeight);
        controller.setOutputBounds(-1, 1);
        mode = DcMotor.RunMode.RUN_TO_POSITION;


        motor = hwMap.get(DcMotorEx.class, LiftConstants.name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(0);
    }

    /** Set power for motor manually (only effective if RUN_WITHOUT_ENCODER is set)*/
    public void setPower(double power) {
        motor.setPower(power);
    }

    /** Set lift mode to specify control mode*/
    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
    }

    /** Get motor power*/
    public double getPower() {
        return motor.getPower();
    }

    public void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Get motor encoder ticks*/
    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /** Set target for PID*/
    public void setTargetPosition(int position) { controller.setTargetPosition(position); }

    /** Update PID controller and motor power*/
    public void update() {
        switch (mode) {
            case RUN_TO_POSITION:
                motor.setPower(controller.update(motor.getCurrentPosition()));
                Log.i("Lift Power", String.format("%f", motor.getPower()));
                Log.i("Lift Error", String.format("%f", controller.getLastError()));
                break;
            case RUN_USING_ENCODER:
//                motor.setPower(0);
                break;
        }
        // Soft limit
//        if (motor.getCurrentPosition() + LiftConstants.endMargin > LiftConstants.maxHeight && motor.getPower() > 0) {
//            motor.setPower(-0.1);
//        } else if (motor.getCurrentPosition() - LiftConstants.endMargin > 0 && motor.getPower() < 0) {
//            motor.setPower(0.1);
//        }
    }

}

