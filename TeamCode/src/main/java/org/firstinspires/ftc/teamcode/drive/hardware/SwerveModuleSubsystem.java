package org.firstinspires.ftc.teamcode.drive.hardware;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotserver.internal.webserver.PingResponse;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class SwerveModuleSubsystem implements DcMotorEx {
    private DcMotorEx driveMotor;
    private CRServo turnServo;
    private Encoder turnEncoder;
    private boolean reversed = false;
    private double lastError = 0;
    private double integralSum = 0;
    private double derivative;
    public final double Kp = DriveConstants.TURN_MODULE_P;
    public final double Ki = DriveConstants.TURN_MODULE_P;
    public final double Kd = DriveConstants.TURN_MODULE_D;

    ElapsedTime timer = new ElapsedTime();

    public SwerveModuleSubsystem(HardwareMap hwMap, String driveName, String turnName, String encoderName) {
        driveMotor = hwMap.get(DcMotorEx.class, driveName);
        turnServo = hwMap.get(CRServo.class, turnName);
        turnEncoder = hwMap.get(Encoder.class, encoderName);
    }

    public void setModuleOrientation(double targetAngle) {
        double currentAngle = getModuleOrientation();
        double turnAng = closestAngle(targetAngle, currentAngle);
        turnServo.setPower(PIDController(turnAng));


    }

    private double PIDController(double error) {
        derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }

    private double closestAngle(double target, double current) {
        double error = Math.toDegrees(target)-Math.toDegrees(current);
        double turnAng = error;
        if (error > 180) {
            turnAng = error - 360;
        } else if (error < -180) {
            turnAng = 360 + error;
        } else if (turnAng > 90) {
            turnAng = turnAng - 180;
            if (reversed){
                reversed = false;
            } else {
                reversed = true;
            }
        } else if (turnAng < -90) {
            turnAng = turnAng + 180;
            if (reversed){
                reversed = false;
            } else {
                reversed = true;
            }
        }
        return turnAng;
    }

    public double getModuleOrientation() {
        return Math.toRadians(turnEncoder.getCurrentPosition()*0.087891);

    }

    @Override
    public MotorConfigurationType getMotorType() {
        return driveMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        driveMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return driveMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return driveMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {

    }

    @Override
    public RunMode getMode() {
        return null;
    }

    @Override
    public void setMotorEnable() {

    }

    @Override
    public void setMotorDisable() {

    }

    @Override
    public boolean isMotorEnabled() {
        return false;
    }

    @Override
    public void setVelocity(double angularRate) {

    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {

    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return 0;
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        driveMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

    }

    @Override
    public void setPositionPIDFCoefficients(double p) {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {
        if (reversed) {
            driveMotor.setPower(-power);
        } else {
            driveMotor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return driveMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
