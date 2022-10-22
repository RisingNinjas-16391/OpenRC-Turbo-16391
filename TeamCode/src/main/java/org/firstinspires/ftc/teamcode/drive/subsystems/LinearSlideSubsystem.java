package org.firstinspires.ftc.teamcode.drive.subsystems;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlideSubsystem {

    public DcMotor LinearSlide;

    public LinearSlideSubsystem(HardwareMap hwMap) {
        LinearSlide = hwMap.get(DcMotorEx.class, "Linear Slide");
        LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearSlide.setPower(0);
    }

    public void setPower(double power) {
        LinearSlide.setPower(power);
    }

    public double getPower() {
        return LinearSlide.getPower();
    }

}

