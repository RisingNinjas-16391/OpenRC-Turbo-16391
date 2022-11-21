package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.drive.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.driveSubsystem.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.liftSubsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.turretSubsystem.TurretSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem drivetrain;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final TurretSubsystem turret;

    private final GamepadEx driverController;

    private final GamepadButton up;
    private final GamepadButton down;
    private final GamepadButton turretToggle;


    public RobotContainer(HardwareMap hwMap) {
        drivetrain = new DrivetrainSubsystem(hwMap);
        lift = new LiftSubsystem(hwMap);
        intake = new IntakeSubsystem(hwMap);
        turret = new TurretSubsystem(hwMap);

        driverController = new GamepadEx(gamepad1);

        up = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);

        down = new GamepadButton(driverController, GamepadKeys.Button.DPAD_DOWN);

        turretToggle = new GamepadButton(driverController, GamepadKeys.Button.A);

        setDefaultCommands();
        configureButtonBindings();
        setAutoCommands();
    }

    private void configureButtonBindings() {
        up.whenPressed(new InstantCommand(lift::incrementHeight));
        down.whenPressed(new InstantCommand(lift::decrementHeight));

        turretToggle.toggleWhenPressed(new TurretCommand(turret, lift));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, lift,
                () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x, () -> gamepad1.right_bumper,
                () -> gamepad1.left_bumper));
        intake.setDefaultCommand(new IntakeCommand(intake, () -> gamepad2.left_bumper));
    }

    public void setAutoCommands() {

    }
}
