package org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem;

import static org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretConstants.*;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    public DcMotorEx motor;
    private static int savedPosition = 0;
    private int offset;
    private boolean toggle = true;
    private MotionProfile profile;
    private double profileStartTime = 0;
    private int targetPosition = 0;
    private final PIDFController controller;
    private final SimpleMotorFeedforward feedforward;
    private final NanoClock clock = NanoClock.system();


    public TurretSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        super();
        motor = hwMap.get(DcMotorEx.class, name);
        // TODO: Turn back to brake mode
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(kPID);
        feedforward = new SimpleMotorFeedforward(kStatic, kV, kA);
        motor.setPower(0);
        offset = savedPosition;
    }

    @Override
    public void periodic() {
        double power;
        savedPosition = motor.getCurrentPosition();
        int currentPosition = getAdjustedPosition(savedPosition);
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getV());
            power = controller.update(currentPosition, motor.getVelocity()) + feedforward.calculate(state.getV(), state.getA());
        } else {
            // just hold the position
            if (controller.getLastError() < tickMargin) {
                return;
            }
            controller.setTargetPosition(targetPosition);
            power = controller.update(currentPosition);
        }
        motor.setPower(power);
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }
    public void setPosition(int position) {
        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(targetPosition, 0, 0, 0);
        targetPosition = Math.min(Math.max(homePos, position), otherSidePos);
        MotionState goal = new MotionState(targetPosition, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();
    }


    public void togglePosition(boolean toggle) {
        this.toggle = toggle;
        setPosition(toggle ? homePos:otherSidePos);
    }

    public void togglePosition() {
        toggle = !toggle;
        togglePosition(toggle);
    }

    public int getAdjustedPosition(int position) {
        return position + offset;
    }

}
