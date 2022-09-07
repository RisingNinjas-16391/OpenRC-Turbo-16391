package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Hardware {
    private final ElapsedTime period = new ElapsedTime();
    /* Public OpMode members. */
    public DrivetrainSubsystem drivetrainSubsystem;
    public ArmSubsystem arm;
    public IntakeSubsystem intake;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        drivetrainSubsystem = new DrivetrainSubsystem(ahwMap);
        arm = new ArmSubsystem(ahwMap);
        intake = new IntakeSubsystem(ahwMap);
    }

    public void displayTelemetry(Telemetry telemetry) { 
        telemetry.addLine("Drive Encoder ticks")
                .addData("Front Left", drivetrainSubsystem.getWheelPositions().get(0))
                .addData("Front Right", drivetrainSubsystem.getWheelPositions().get(3))
                .addData("Back Left", drivetrainSubsystem.getWheelPositions().get(1))
                .addData("Back Right", drivetrainSubsystem.getWheelPositions().get(2));
        telemetry.update();
    }
}