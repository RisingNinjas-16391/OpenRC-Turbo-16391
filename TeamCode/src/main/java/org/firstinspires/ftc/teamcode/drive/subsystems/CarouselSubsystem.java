package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSubsystem {

    public DcMotor Carousel;

    public CarouselSubsystem(HardwareMap hwMap) {
        Carousel = hwMap.get(DcMotorEx.class, "Carousel");
        Carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        Carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        Carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Carousel.setPower(0);
    }

    public void setPower(double power) {
        Carousel.setPower(power);
    }

}
