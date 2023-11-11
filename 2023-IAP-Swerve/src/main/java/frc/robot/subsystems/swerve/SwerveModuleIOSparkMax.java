// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private CANSparkMax driveSparkMax;
    private CANSparkMax turnSparkMax;

    private SparkMaxPIDController drivePID;
    private PIDController turnPID;

    private RelativeEncoder driveEncoder;
    private CANCoder turnEncoder;

    // Object to hold swerve module state
    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0));

    // Might not need offset if using CANCoders
    // private double angularOffset = 0.0;

    private int num = 0;

    public SwerveModuleIOSparkMax(int num, int driveID, int turnID, int turnCANCoderID, double turnEncoderOffset) {

        turnEncoder = new CANCoder(turnCANCoderID);

        // Construct CANSparkMaxes
        driveSparkMax = new CANSparkMax(driveID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(turnID, MotorType.kBrushless);

        turnPID = new PIDController(0, 0, 0);

        // Initialize encoder and PID controller
        driveEncoder = driveSparkMax.getEncoder();
        drivePID = driveSparkMax.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        // Reset to defaults
        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        // Set conversion factors
        driveEncoder.setPositionConversionFactor(0.0);
        driveEncoder.setVelocityConversionFactor(0.0);

        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        drivePID.setP(1.0);
        drivePID.setI(1.0);
        drivePID.setD(1.0);
        drivePID.setFF(1.0);
        drivePID.setOutputRange(-1, 1);

        driveSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveSparkMax.setSmartCurrentLimit(1);
        turnSparkMax.setSmartCurrentLimit(1);

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);

        // Only useful for encoder
        // turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        driveEncoder.setPosition(0);

        // Set position of encoder to absolute mode
        turnEncoder.setPositionToAbsolute();
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configMagnetOffset(turnEncoderOffset);

        turnPID.setP(1.0);
        turnPID.setI(1.0);
        turnPID.setD(1.0);

        // Continous input jumping from -PI to PI
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        this.state.angle = new Rotation2d(getTurnPositionInRad());

        this.num = num;

    }

    public double getTurnPositionInRad() {
        // Divide by 1.0, as CANCoder has direct measurement of output
        return ((turnEncoder.getAbsolutePosition() / 4096.0) * 2 * Math.PI)
                / 1.0;
    }

    public void setDesiredState(SwerveModuleState state) {
        // Optimize state so that movement is minimized
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPositionInRad()));
        
        SmartDashboard.putNumber("Setpoint Drive Vel #" + this.num, state.speedMetersPerSecond);

        // Cap setpoints at max speeds for safety
        state.speedMetersPerSecond = MathUtil.clamp(state.speedMetersPerSecond, -1, 1);

        // Set reference of drive motor's PIDF internally in SPARK MAX
        // This automagically updates at a 1 KHz rate
        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        // Set SmartDashboard telemetry - this runs once every 20 ms
        SmartDashboard.putNumber("Closed Loop Drive Power #" + this.num, driveSparkMax.get());
        SmartDashboard.putNumber("Drive Velocity #" + this.num, state.speedMetersPerSecond);

        // Calculate PID in motor power (from -1.0 to 1.0)
        double turnOutput = MathUtil.clamp(this.turnPID.calculate(getTurnPositionInRad()), -1.0, 1.0);

        // Set voltage of SPARK MAX
        turnSparkMax.setVoltage(turnOutput * 12.0);

        // Set internal state as passed-in state
        this.state = state;


    }

    public SwerveModuleState getSetpointModuleState() {
        // Returns module state
        return this.state;
    }

    public SwerveModuleState getActualModuleState() {
        double velocity = this.driveEncoder.getVelocity();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
    }

    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    public void setDriveBrakeMode(boolean enable) {
        if (enable) {
            driveSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            driveSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setTurnBrakeMode(boolean enable) {
        if (enable) {
            turnSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            turnSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModulePosition(position, new Rotation2d(rotation));
    }

    public void resetEncoders() {
        // Resets only drive encoder
        driveEncoder.setPosition(0.0);
    }

    public int getNum() {
        return num;
    }
}