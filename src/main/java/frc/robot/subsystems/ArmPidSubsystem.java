// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmPidSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax motor = new CANSparkMax(ArmConstants.MOTOR_PORT, MotorType.kBrushless);
      new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder encoder = this.motor.getEncoder();

  /** Create a new ArmSubsystem. */
  public ArmPidSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.PCONSTANT,
            ArmConstants.kI,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY_RAD_PER_SECOND,
                ArmConstants.MAX_ACCELERATION_RAD_PER_SEC_SQUARED)),
        0);

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setVoltage(0.0);
    // Start arm at rest in neutral position
    // setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setPoint) {
    this.motor.setVoltage(output);
    SmartDashboard.putNumber("voltage", output);
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("armPosition", this.encoder.getPosition());
    return this.encoder.getPosition() + ArmConstants.ARM_OFFSET_RADS;
  }
}
