// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmPIDSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax motor =
      new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder encoder = this.motor.getEncoder();
  private final ArmFeedforward feedforward =
  new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmPIDSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    this.motor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("armPosition", this.encoder.getPosition());
    System.out.println(this.encoder.getPosition());
    return this.encoder.getPosition() + ArmConstants.kArmOffsetRads;
  }

  public void up() {
    // need to write this method

  }

  public void down() {
    // need to write this method

  }
}
