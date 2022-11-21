package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.commands.AutoCommand1;
import org.firstinspires.ftc.teamcode.drive.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final TurretSubsystem turret;
    private final AprilTagSubsystem aprilTagDetector;

    private final GamepadEx driverController = new GamepadEx(gamepad1);
    private final GamepadEx operatorController = new GamepadEx(gamepad2);

    private final GamepadButton up = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);
    private final GamepadButton down = new GamepadButton(driverController, GamepadKeys.Button.DPAD_DOWN);
    private final GamepadButton slowMode = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
    private final GamepadButton turboMode = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
    private final GamepadButton lockRotation = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_STICK_BUTTON);

    private final GamepadButton dropCone = new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER);
    private final GamepadButton turretToggle = new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER);

    /**
     * Teleop Constructor
     */
    public RobotContainer(HardwareMap hwMap) {
        drivetrain = new DrivetrainSubsystem(hwMap);
        lift = new LiftSubsystem(hwMap);
        intake = new IntakeSubsystem(hwMap);
        turret = new TurretSubsystem(hwMap);
        aprilTagDetector = new AprilTagSubsystem(hwMap);

        setDefaultCommands();
        configureButtonBindings();

    }

    /**
     * Autonomous Constructor
     */
    public RobotContainer(HardwareMap hwMap, int autoNum) {
        drivetrain = new DrivetrainSubsystem(hwMap);
        lift = new LiftSubsystem(hwMap);
        intake = new IntakeSubsystem(hwMap);
        turret = new TurretSubsystem(hwMap);
        aprilTagDetector = new AprilTagSubsystem(hwMap);

        setAutoCommands(autoNum);
    }

    private void configureButtonBindings() {
        // Driver bindings
        // Speed mode button binding
        slowMode.whenPressed(() -> drivetrain.setSpeedMultiplier(0.6));
        turboMode.whenActive(() -> drivetrain.setSpeedMultiplier(1));
        slowMode.negate().and(turboMode.negate()).whenActive(() -> drivetrain.setSpeedMultiplier(0.7));

        // Operator bindings
        dropCone.whenPressed(new InstantCommand(intake::unfeed, intake));
        turretToggle.whenPressed(new TurretCommand(turret, lift::getCurrentHeight));
        up.whenPressed(lift::incrementHeight);
        down.whenPressed(lift::decrementHeight);
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, lift::getCurrentHeight,
                driverController::getLeftY, driverController::getLeftX,
                driverController::getRightX));
        intake.setDefaultCommand(new InstantCommand(intake::feed));
    }

    public void setAutoCommands(int chooser) {
        Command Auto1 = new AutoCommand1(drivetrain, lift, intake, aprilTagDetector);
        switch (chooser) {
            case 0:
                Auto1.schedule();
                break;
            case 1:
                break;
        }

    }
}
