package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.commands.AutoCommandFiveCone;
import org.firstinspires.ftc.teamcode.drive.commands.AutoCommandOneCone;
import org.firstinspires.ftc.teamcode.drive.commands.AutoCommandPark;
import org.firstinspires.ftc.teamcode.drive.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.FieldCentricDrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LockedHeadingDrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretLockCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretToggleCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.aprilTagSubsystem.aprilTagDetector.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

public class

RobotContainer {
    private final DrivetrainSubsystem drivetrain;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
//    private final TurretSubsystem turret;
    private final AprilTagSubsystem aprilTagDetector;

    private final GamepadEx driverController;
    private final GamepadEx operatorController;

    private final GamepadButton up;
    private final GamepadButton down;
    private final GamepadButton slowMode;
    private final GamepadButton turboMode;
    private final GamepadButton threeAbove;
//    private final GamepadButton lockRotation;

    private final GamepadButton dropConeD;
    private final GamepadButton dropConeO;
    private final GamepadButton feedCone;
//    private final GamepadButton turretToggle;
    private final GamepadButton scoringHeight;
    private final GamepadButton resetPose;

    /**
     * Teleop Constructor
     */
    public RobotContainer(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        drivetrain = new DrivetrainSubsystem(hwMap, telemetry);
        lift = new LiftSubsystem(hwMap, telemetry);
        intake = new IntakeSubsystem(hwMap);
//        turret = new TurretSubsystem(hwMap, telemetry);
        aprilTagDetector = new AprilTagSubsystem(hwMap, telemetry);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);


        // Driver Controls
        slowMode = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        turboMode = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
//        lockRotation = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        dropConeD = new GamepadButton(driverController, GamepadKeys.Button.A);
        threeAbove = new GamepadButton(operatorController, GamepadKeys.Button.X);

        // Operator Controls
        dropConeO = new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER);
        feedCone = new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER);
//        turretToggle = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_LEFT);
        up = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_DOWN);
        scoringHeight = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_RIGHT);
        resetPose = new GamepadButton(operatorController, GamepadKeys.Button.START);

        setDefaultCommands();
        configureButtonBindings();
        telemetry.setAutoClear(true);
    }

    /**
     * Autonomous Constructor
     */
    public RobotContainer(HardwareMap hwMap, int autoNum, Telemetry telemetry) {
        drivetrain = new DrivetrainSubsystem(hwMap, telemetry);
        lift = new LiftSubsystem(hwMap, telemetry);
        intake = new IntakeSubsystem(hwMap);
//        turret = new TurretSubsystem(hwMap, telemetry);
        aprilTagDetector = new AprilTagSubsystem(hwMap, telemetry);

        driverController = null;
        operatorController = null;

        slowMode = null;
        turboMode = null;
        threeAbove = null;
//        lockRotation = null;

        up = null;
        down = null;
        dropConeD = null;
        dropConeO = null;
        feedCone = null;
//        turretToggle = null;
        scoringHeight = null;
        resetPose = null;

        setAutoCommands(autoNum, telemetry);
    }

    private void configureButtonBindings() {
        // Driver bindings
        // Speed mode button binding
        slowMode.whileHeld(() -> drivetrain.setSpeedMultiplier(0.5));
        turboMode.whileHeld(() -> drivetrain.setSpeedMultiplier(1));
        slowMode.negate().and(turboMode.negate()).whenActive(() -> drivetrain.setSpeedMultiplier(0.7));
//        lockRotation.toggleWhenPressed(new LockedHeadingDrivetrainCommand(drivetrain, lift::getDriveMultiplier, driverController::getLeftY, driverController::getLeftX, drivetrain.getPoseEstimate().getHeading()));
        dropConeD.whileHeld(new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED).perpetually());

        // Operator bindings
        dropConeO.whileHeld(new IntakeCommand(intake, IntakeSubsystem.Direction.UNFEED).perpetually());
        feedCone.whileHeld(new IntakeCommand(intake, IntakeSubsystem.Direction.FEED).perpetually());
//        turretToggle.whenPressed(new TurretToggleCommand(turret, lift::getCurrentHeight));
        up.whenPressed(lift::incrementHeight);
        down.whenPressed(lift::decrementHeight);
        scoringHeight.whenPressed(lift::scoreHeight);
        threeAbove.whenPressed(lift::beaconHeight);
        resetPose.whenPressed((() -> drivetrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)))));
    }

    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, lift::getDriveMultiplier,
                driverController::getLeftY, driverController::getLeftX,
                driverController::getRightX));

        intake.setDefaultCommand(new IntakeCommand(intake, IntakeSubsystem.Direction.FEED).perpetually());
//        turret.setDefaultCommand(new TurretLockCommand(turret));
    }

    private void setAutoCommands(int chooser, Telemetry telemetry) {
        Command AutoPark = new AutoCommandPark(drivetrain, lift, intake, aprilTagDetector, telemetry);
        Command AutoOneCone = new AutoCommandOneCone(drivetrain, lift, intake, aprilTagDetector, telemetry);

        switch (chooser) {
            case 0:
                AutoPark.schedule();
                break;
            case 1:
                AutoOneCone.schedule();
                break;
        }

    }
}
