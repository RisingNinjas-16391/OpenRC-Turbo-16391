package org.firstinspires.ftc.teamcode.drive.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Hardware {
    private final ElapsedTime period = new ElapsedTime();
    /* Public OpMode members. */
    public DrivetrainSubsystem drivetrainSubsystem;
    public LiftSubsystem linearSlide;
    public IntakeSubsystem intake;
//    public TurretSubsystem turret;

    /* Constructor */
    public Hardware() {

    }

    /** Initialize standard Hardware interfaces */
    public void init(@NonNull HardwareMap ahwMap) {
        // Save reference to Hardware map
        drivetrainSubsystem = new DrivetrainSubsystem(ahwMap);
        linearSlide = new LiftSubsystem(ahwMap);
        intake = new IntakeSubsystem(ahwMap);
//        turret = new TurretSubsystem(ahwMap);
    }
    /** Runs the update method in all subsystems */
    public void update() {
        drivetrainSubsystem.update();
        linearSlide.update();
    }

    /** Displays important robot information on telemetry*/
    public void displayTelemetry(@NonNull Telemetry telemetry) {
        telemetry.addLine("Drive Encoder ticks")
                .addData("Front Left", drivetrainSubsystem.getWheelPositions().get(0))
                .addData("Front Right", drivetrainSubsystem.getWheelPositions().get(3))
                .addData("Back Left", drivetrainSubsystem.getWheelPositions().get(1))
                .addData("Back Right", drivetrainSubsystem.getWheelPositions().get(2));

        telemetry.addLine("Linear Slide ticks")
                .addData("Slide", linearSlide.getCurrentPosition());

//        telemetry.addLine("Turret Ticks")
//                .addData("Turret", turret.getCurrentPosition());
        telemetry.update();
    }
}