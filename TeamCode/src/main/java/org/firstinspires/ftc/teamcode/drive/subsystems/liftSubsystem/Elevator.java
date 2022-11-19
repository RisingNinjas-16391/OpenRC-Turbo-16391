package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_JERK;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MOTOR_CONFIG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.SPOOL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kPID;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.name;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.DIRECTION;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Elevator {

    private DcMotorEx motor;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;


    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public Elevator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so "up" is positive
        motor.setDirection(DIRECTION);

        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDFController(kPID, kV, kA, kStatic);
        // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
        ElevatorFeedforward feedforward = new ElevatorFeedforward(
                kStatic, kG, kV, kA
        );
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredHeight, 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.desiredHeight = height;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentHeight, state.getV());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredHeight);
            power = controller.update(currentHeight);
        }
        setPower(power);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

}