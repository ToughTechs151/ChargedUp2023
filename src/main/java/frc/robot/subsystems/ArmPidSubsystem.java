// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmRetractCommand;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmPidSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax armMotor =
      new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder encoder = armMotor.getEncoder();
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private final Spark blinkinSpark = new Spark(Constants.BLIKIN_SPARK_PORT);
  private double blinkinVoltage = Constants.BLINKIN_DARK_GREEN;
  private RobotContainer robotContainer = null;

  /** Create a new ArmSubsystem. */
  public ArmPidSubsystem(RobotContainer robotContainer) {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    this.robotContainer = robotContainer;
    this.disable();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setVoltage(0.0);
    blinkinSpark.set(blinkinVoltage);
    // Start arm at rest in neutral position
    // setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    // double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    SmartDashboard.putNumber("Output voltage", output);
    armMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    double position = encoder.getPosition();
    SmartDashboard.putNumber("ARM Position", position);
    SmartDashboard.putNumber("ARM motor velocity", encoder.getVelocity());
    setLED(position);
    retractArm(position);
    return position + ArmConstants.kArmOffsetRads;
  }

  /**
   * Set voltage to the ARM motor
   * @param voltage
   */
  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  /**
   * This method set the RevRobotics Blinkin LED to flash based on the ARM angle
   *
   * @param position
   */
  private void setLED(double position) {
    // Change the Blikin LED based on the angle of the arm
    if (position >= ArmConstants.ARM_RED_ZONE) {
      blinkinVoltage = Constants.BLINKIN_RED;
    } else {
      blinkinVoltage = Constants.BLINKIN_DARK_GREEN;
    }
    blinkinSpark.set(blinkinVoltage);
  }

  /**
   * retract the ARM if it reaches the red zone
   *
   * @param position
   */
  private void retractArm(double position) {

    if (position >= ArmConstants.ARM_RED_ZONE) {
      (new ArmRetractCommand(this.robotContainer.getArmSubsystem())).schedule();
    }
  }
}
