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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
//import frc.robot.Constants.ModuleConstants;

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
    }
}