package org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.BOTTOM_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.DIRECTION;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.FEED_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.HIGH_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.LOW_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_JERK;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MID_POS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.MOTOR_CONFIG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.SPOOL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kG;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kPID;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftConstants.name;

import com.acmerobotics.dashboard.config.Config;
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

/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class LiftSubsystem extends SubsystemBase {

    private DcMotorEx lift;
    private PIDFController controller;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(
            kStatic, kG, kV, kA
    );
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;
    private int heightIndex;

    public LiftSubsystem(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, name);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DIRECTION);

        controller = new PIDFController(kPID);

        // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA

        offset = lift.getCurrentPosition();

        heightIndex = 0;
    }

    @Override
    public void periodic() {
        double power;
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentHeight, state.getV()) + feedforward.calculate(state.getV(), state.getA());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredHeight);
            power = controller.update(currentHeight);
        }
        lift.setPower(power);

        telemetry.addLine("Linear Slide ticks")
                .addData("slide", lift.getCurrentPosition());

        telemetry.addLine("Linear Slide power")
                .addData("slide", lift.getPower());

        telemetry.addLine("Turret Encoder Position")
                .addData("Ticks: ", lift.getCurrentPosition());

        indexToHeight();
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
        return encoderTicksToInches(lift.getCurrentPosition() - offset);
    }

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public void incrementHeight() {
        heightIndex++;
    }

    public void decrementHeight() {
        heightIndex--;
    }

    private void indexToHeight() {
        switch (heightIndex) {
            case 0:
                this.setHeight(BOTTOM_POS);
                break;
            case 1:
                this.setHeight(FEED_POS);
                break;
            case 2:
                this.setHeight(LOW_POS);
                break;
            case 3:
                this.setHeight(MID_POS);
                break;
            case 4:
                this.setHeight(HIGH_POS);
        }
    }

    public void indexToHeight(int height) {
        switch (height) {
            case 0:
                this.setHeight(BOTTOM_POS);
                break;
            case 1:
                this.setHeight(FEED_POS);
                break;
            case 2:
                this.setHeight(LOW_POS);
                break;
            case 3:
                this.setHeight(MID_POS);
                break;
            case 4:
                this.setHeight(HIGH_POS);
        }
    }
}