package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.BOTTOM_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.DIRECTION;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.EX_HIGH_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.FEED_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.HIGH_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.Inches_above;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.LOW_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_JERK;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MID_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MOTOR_CONFIG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.SCORE_ADJ;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.SPOOL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.heightThreshold;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kPID;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.name;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.simpleMode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
public class LiftSubsystem extends SubsystemBase {

    private static int savedPosition = 0;
    private final DcMotorEx motor;
    private final PIDFController controller;
    private final Telemetry telemetry;
    private final NanoClock clock = NanoClock.system();
    private final int offset;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(
            kStatic, kG, kV, kA
    );
    private MotionProfile profile;
    private double profileStartTime = 0;
    private double targetHeight = 0;
    private int heightIndex;
    private DcMotor.RunMode mode;
    private boolean seeking = false;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super();
        this.telemetry = telemetry;

        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setDirection(DIRECTION);

        controller = new PIDFController(kPID);
        mode = DcMotor.RunMode.RUN_TO_POSITION;
        motor.setPower(0);

        // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA

        heightIndex = 0;

        offset = savedPosition;

    }

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    @Override
    public void periodic() {
        double currentHeight = getCurrentHeight();

        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            double power;

            if (isBusy()) {

                if (simpleMode || profile == null) {
                    power = currentHeight < targetHeight ? 1 : -1;   // Bang Bang control
                } else {  // following a profile
                    double time = clock.seconds() - profileStartTime;
                    MotionState state = profile.get(time);
                    controller.setTargetPosition(state.getX());
                    controller.setTargetVelocity(state.getV());
                    controller.setTargetAcceleration(state.getV());
                    power = controller.update(currentHeight, motor.getVelocity()) + feedforward.calculate(state.getV(), state.getA());
                }
            } else {
                // just hold the position
                controller.setTargetPosition(targetHeight);
                power = controller.update(currentHeight);
                if (Math.abs(controller.getLastError()) > 5) {
                    power += feedforward.ks * Math.signum(power);
                }
            }
            motor.setPower(power);
        }

        telemetry.addLine("Linear Slide Level")
                .addData("slide", heightIndex);
    }

    public boolean isBusy() {
        if (!simpleMode)
            return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
        else {
            seeking = seeking || Math.abs(getCurrentHeight() - targetHeight) > heightThreshold;
            return seeking;
        }
    }

    public void setHeight(double height) {
        targetHeight = Math.min(Math.max(0, height), MAX_HEIGHT);
        if (!simpleMode) {
            double time = clock.seconds() - profileStartTime;
            MotionState start = isBusy() && profile != null ? profile.get(time) : new MotionState(getCurrentHeight(), 0, 0, 0);
            MotionState goal = new MotionState(targetHeight, 0, 0, 0);
            try {
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
                );
            } catch (Exception e) {
                System.out.println(e);
            }
            profileStartTime = clock.seconds();
        } else {
            seeking = true;
        }
    }

    public double getCurrentHeight() {
        savedPosition = getAdjustedPosition(motor.getCurrentPosition());
        return encoderTicksToInches(savedPosition);
    }

    public void incrementHeight() {
        heightIndex++;
        indexToHeight();
    }

    public void decrementHeight() {
        heightIndex--;
        indexToHeight();
    }

    public void beaconHeight() {
        setHeight(Inches_above);
    }

    public void scoreHeight() {
        if (heightIndex > 1) {
            setHeight(targetHeight - SCORE_ADJ);
        }
    }

    public double getDriveMultiplier() {
        double currentHeight = getCurrentHeight();
        double correction = 1;
        if (currentHeight > 0) {
            correction = 1 - (currentHeight / LiftConstants.MAX_HEIGHT);
            correction /= 2;
            correction += 0.5;
        }
        return correction;
    }

    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
        if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void indexToHeight() {
        if (heightIndex > 5) {
            heightIndex = 5;
        } else if (heightIndex < 0) {
            heightIndex = 0;
        }
        indexToHeight(heightIndex);
    }

    public void indexToHeight(int height) {
        switch (height) {
            case 0:
                setHeight(BOTTOM_POS);
                break;
            case 1:
                setHeight(FEED_POS);
                break;
            case 2:
                setHeight(LOW_POS);
                break;
            case 3:
                setHeight(MID_POS);
                break;
            case 4:
                setHeight(HIGH_POS);
                break;
            case 5:
                setHeight(EX_HIGH_POS);
                break;
        }
        heightIndex = height;
    }

    public int getAdjustedPosition(int position) {
        return position + offset;
    }
}