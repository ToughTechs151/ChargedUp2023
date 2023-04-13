// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPidSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmUpCommand extends InstantCommand {

  private ArmPidSubsystem armSubsystem;
  /**
   * @param arm
   * @param arm ArmPIDSubsystem
   */

  private double lastarmHigh;

  public ArmUpCommand(ArmPidSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    armSubsystem = arm;
    armSubsystem.enable();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastarmHigh = RobotContainer.getArmHigh();
    armSubsystem.setGoal(new State(lastarmHigh, ArmConstants.ARM_VELOCITY));
  }
  
  @Override
  public void execute() {
    if (lastarmHigh != RobotContainer.getArmHigh()) {
      lastarmHigh = RobotContainer.getArmHigh();
      SmartDashboard.putString("ArmUpCommand", "setGoal");
      armSubsystem.setGoal(new State(lastarmHigh, ArmConstants.ARM_VELOCITY));
    }
  }
}
