// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPidSubsystem;

public class ArmMoveUpCommand extends CommandBase {
  private ArmPidSubsystem armPidSubsystem;

  /** Creates a new ArmMoveUpCommand. */
  public ArmMoveUpCommand(ArmPidSubsystem armPidSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armPidSubsystem = armPidSubsystem;
    addRequirements(armPidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPidSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armPidSubsystem.getMeasurement() <= 0.0) {
      armPidSubsystem.setVoltage(0.0);
    } else {
      armPidSubsystem.setVoltage(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armPidSubsystem.setVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    armPidSubsystem.enable();
    return false;
  }
}