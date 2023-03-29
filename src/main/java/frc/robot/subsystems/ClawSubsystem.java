// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private final DoubleSolenoid doubleSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.CLAW_SOLENOID_KFORWARD,
          Constants.CLAW_SOLENOID_KREVERSE);

  private final WPI_TalonSRX clawMotor =
      new WPI_TalonSRX(Constants.CLAW_MOTOR);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    /* Default constructor */
    this.clawMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open() {
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void runClaw(double direction) {
    clawMotor.set(ControlMode.PercentOutput,direction);
  }
}
