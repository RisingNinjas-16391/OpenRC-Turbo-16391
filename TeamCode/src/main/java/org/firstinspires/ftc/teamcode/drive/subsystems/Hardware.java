package org.firstinspires.ftc.teamcode.drive.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

import java.util.function.BooleanSupplier;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Hardware {
    private final ElapsedTime period = new ElapsedTime();
    /* Public OpMode members. */
    public DrivetrainSubsystem drivetrain;
    public LiftSubsystem slide;
    public IntakeSubsystem intake;
    public TurretSubsystem turret;
    private final BooleanSupplier isOpmodeRunning;
    private final BooleanSupplier isStopRequested;

    /* Constructor */
    public Hardware() {
        this.isOpmodeRunning = () -> false;
        this.isStopRequested = () -> true;
    }

    public Hardware(BooleanSupplier isOpmodeRunning, BooleanSupplier isStopRequested) {
        this.isOpmodeRunning = isOpmodeRunning;
        this.isStopRequested = isStopRequested;
    }

    /** Initialize standard Hardware interfaces */
    public void init(@NonNull HardwareMap ahwMap) {
        // Save reference to Hardware map
        drivetrain = new DrivetrainSubsystem(ahwMap);
        slide = new LiftSubsystem(ahwMap);
        intake = new IntakeSubsystem(ahwMap);
        turret = new TurretSubsystem(ahwMap);
    }
    /** Runs the update method in all subsystems */
    public void update() {
        drivetrain.update();
        slide.update();
    }

    public void finishTrajectory(int timeout) {
        ElapsedTime timer = new ElapsedTime();
        while (isOpmodeRunning.getAsBoolean() && !isStopRequested.getAsBoolean() && this.drivetrain.isBusy() && timer.time() < timeout) {
            this.update();
        }
    }

    public void finishLift(int timeout) {
        ElapsedTime timer = new ElapsedTime();
        while (isOpmodeRunning.getAsBoolean() && !isStopRequested.getAsBoolean() && this.slide.isBusy() && timer.time() < timeout) {
            this.update();
        }
    }

    public void wait(int timeout) {
        ElapsedTime timer = new ElapsedTime();
        while (isOpmodeRunning.getAsBoolean() && !isStopRequested.getAsBoolean() && timer.time() < timeout) {
            this.update();
        }
    }

    /** Displays important robot information on telemetry*/
    public void displayTelemetry(@NonNull Telemetry telemetry) {
        telemetry.addLine("Drive Encoder ticks")
                .addData("Front Left", drivetrain.getWheelPositions().get(0))
                .addData("Front Right", drivetrain.getWheelPositions().get(3))
                .addData("Back Left", drivetrain.getWheelPositions().get(1))
                .addData("Back Right", drivetrain.getWheelPositions().get(2));

        telemetry.addLine("Linear Slide ticks")
                .addData("slide", slide.getCurrentPosition());

        telemetry.addLine("Linear Slide power")
                .addData("slide", slide.getPower());

        telemetry.addLine("Turret Encoder Position")
                .addData("Ticks: ", turret.getCurrentPosition());

//        telemetry.addLine("Intake Power")
//                .addData("intake", intake.getPower());
//
//        telemetry.addLine("Turret Ticks")
//                .addData("Turret", turret.getCurrentPosition());

        telemetry.update();
    }
}