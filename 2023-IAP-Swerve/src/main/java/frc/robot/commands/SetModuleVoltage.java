// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SingularModule;

public class SetModuleVoltage extends CommandBase {

  private SingularModule module;
  private Joystick joy;

  /** Creates a new SetModuleVoltage. */
  public SetModuleVoltage(SingularModule module, Joystick joy) {
    
    this.module = module;
    this.joy = joy;

    addRequirements(module);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    module.module.setDriveBrakeMode(true);
    module.module.setDriveVoltage(0.0);
    module.module.setTurnVoltage(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.module.setDriveVoltage(joy.getRawAxis(1));
    module.module.setTurnVoltage(joy.getRawAxis(5));

    if(joy.getRawButton(1)) {

      module.module.setTurnVoltage(1.0);

    } else if(joy.getRawButton(2)) {

      module.module.setTurnVoltage(4.0);

    } else if(joy.getRawButton(3)) {

      module.module.setTurnVoltage(8.0);

    } else if(joy.getRawButton(4)) {

      module.module.setTurnVoltage(12.0);

    } else if(joy.getRawButton(5)) {

      module.module.setDriveVoltage(1.0);

    } else if(joy.getRawButton(6)) {

      module.module.setDriveVoltage(4.0);

    } else if(joy.getRawButton(7)) {

      module.module.setDriveVoltage(8.0);

    } else if(joy.getRawButton(8)) {

      module.module.setDriveVoltage(12.0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.module.setDriveBrakeMode(true);
    module.module.setDriveVoltage(0);
    module.module.setTurnVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
