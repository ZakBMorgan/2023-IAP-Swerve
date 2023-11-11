// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingularModule extends SubsystemBase {
  /** Creates a new SingularModule. */

  public SwerveModuleIO module;

  public SingularModule(SwerveModuleIO module) {
    this.module = module;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Real Turn Pos #" + module.getNum(), Units.radiansToDegrees(module.getTurnPositionInRad()));
    SmartDashboard.putNumber("Raw Turn Pos #" + module.getNum(), module.getTurnPositionInRad());
    SmartDashboard.putNumber("Real Velocity #" + module.getNum(), module.getActualModuleState().speedMetersPerSecond);
  }
}
