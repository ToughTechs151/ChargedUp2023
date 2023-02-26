// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more

public class ArmRetractCommand extends InstantCommand {

  private ArmSubsystem armSystem;

  public ArmRetractCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    armSystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSystem.back();
  }
}
