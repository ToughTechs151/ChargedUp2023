// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPidSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmScoreLowCommand extends InstantCommand {
  private ArmPidSubsystem armSubsystem;

  public ArmScoreLowCommand(ArmPidSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    armSubsystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setGoalPosition(new State(ArmConstants.ARM_SCORE_LOW_POSITION_RADS, 0));
    armSubsystem.enable();
  }
}
