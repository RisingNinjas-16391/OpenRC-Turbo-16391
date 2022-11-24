package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.commands.AutoCommand1;
import org.firstinspires.ftc.teamcode.drive.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LockedHeadingDrivetrainCommand;
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

    private final GamepadEx driverController;
    private final GamepadEx operatorController;

    private final GamepadButton up;
    private final GamepadButton down;
    private final GamepadButton slowMode;
    private final GamepadButton turboMode;
    private final GamepadButton lockRotation;

    private final GamepadButton dropCone;
    private final GamepadButton turretToggle;
    private final GamepadButton scoringHeight;

    /**
     * Teleop Constructor
     */
    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drivetrain = new DrivetrainSubsystem(hwMap, telemetry);
        lift = new LiftSubsystem(hwMap, telemetry);
        intake = new IntakeSubsystem(hwMap, telemetry);
        turret = new TurretSubsystem(hwMap, telemetry);
        aprilTagDetector = new AprilTagSubsystem(hwMap, telemetry);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);


        slowMode = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        turboMode = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        lockRotation = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        dropCone = new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER);
        turretToggle = new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER);
        up = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_DOWN);
        scoringHeight = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_LEFT);

        setDefaultCommands();
        configureButtonBindings();

    }

    /**
     * Autonomous Constructor
     */
    public RobotContainer(HardwareMap hwMap, int autoNum, Telemetry telemetry) {
        drivetrain = new DrivetrainSubsystem(hwMap, telemetry);
        lift = new LiftSubsystem(hwMap, telemetry);
        intake = new IntakeSubsystem(hwMap, telemetry);
        turret = new TurretSubsystem(hwMap, telemetry);
        aprilTagDetector = new AprilTagSubsystem(hwMap, telemetry);

        driverController = null;
        operatorController = null;

        slowMode = null;
        turboMode = null;
        lockRotation = null;

        up = null;
        down = null;
        dropCone = null;
        turretToggle = null;
        scoringHeight = null;

        setAutoCommands(autoNum);
    }

    private void configureButtonBindings() {
        // Driver bindings
        // Speed mode button binding
        slowMode.whenPressed(() -> drivetrain.setSpeedMultiplier(0.6));
        turboMode.whenActive(() -> drivetrain.setSpeedMultiplier(1));
        slowMode.negate().and(turboMode.negate()).whenActive(() -> drivetrain.setSpeedMultiplier(0.7));
        lockRotation.toggleWhenPressed(new LockedHeadingDrivetrainCommand(drivetrain, lift::getDriveMultiplier, driverController::getLeftY, driverController::getLeftX, drivetrain.getPoseEstimate().getHeading()));

        // Operator bindings
        dropCone.whileActiveOnce(new IntakeCommand(intake, false));
        turretToggle.whenPressed(new TurretCommand(turret, lift::getCurrentHeight));
        up.whenPressed(lift::incrementHeight);
        down.whenPressed(lift::decrementHeight);
        scoringHeight.whenPressed(lift::scoreHeight);
        scoringHeight.whenReleased(() -> lift.indexToHeight());

    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new FieldCentricDrivetrainCommand(drivetrain, lift::getDriveMultiplier,
                driverController::getLeftY, driverController::getLeftX,
                driverController::getRightX));

        intake.setDefaultCommand(new IntakeCommand(intake, true));
    }

    private void setAutoCommands(int chooser) {
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
