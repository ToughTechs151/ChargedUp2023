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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
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
  private TrapezoidProfile.State lastGoal;

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

    encoder.setPositionConversionFactor(ArmConstants.ARM_RAD_PER_ENCODER_ROTATION);
    encoder.setVelocityConversionFactor(ArmConstants.RPM_TO_RAD_PER_SEC);
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Arm Enabled", m_enabled);
    SmartDashboard.putNumber("Arm Angle", Units.radiansToDegrees(getMeasurement()));
    SmartDashboard.putNumber("Arm Velocity", Units.radiansToDegrees(encoder.getVelocity()));
    SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    // double newFeedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // voltageCommand = output + newFeedforward;
    // setVoltage(voltageCommand);
    setVoltage(output);

    SmartDashboard.putNumber("Arm SetPt Pos", Units.radiansToDegrees(setpoint.position));
    SmartDashboard.putNumber("Arm SetPt Vel", Units.radiansToDegrees(setpoint.velocity));
    // SmartDashboard.putNumber("Arm Feedforward", newFeedforward);
    SmartDashboard.putNumber("Arm PID output", output);
    // SmartDashboard.putNumber("Arm Command Voltage", voltageCommand);

  }

  // This function is used for feedback in the PID controller.
  // Units must be radians with positive oriented up and 0.0 at horizontal position.
  @Override
  public double getMeasurement() {
    double position = -encoder.getPosition() + ArmConstants.ARM_OFFSET_RADS;
    setLED(position);
    retractArm(position);
    return position;
  }

  // Convert the measured position to arm angle relative to the up position.
  // Positive is down.
  public double getAngle(double position) {

    return position;
  }
  public void setVoltage(double voltage){
    armMotor.setVoltage(voltage);
  }

  /**
   * This method set the RevRobotics Blinkin LED to flash based on the ARM angle
   *
   * @param position
   */
  private void setLED(double position) {
    // Change the Blikin LED based on the angle of the arm
    if (position >= ArmConstants.ARM_RED_ZONE_RADS) {
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

    if (position >= ArmConstants.ARM_RED_ZONE_RADS) {
      (new ArmRetractCommand(this.robotContainer.getArmSubsystem())).schedule();
    }
  }

  @Override
  public void setGoal(TrapezoidProfile.State goal) {
    lastGoal = goal;
    super.setGoal(goal);    
    SmartDashboard.putNumber("ARM Goal Pos", goal.position);
    SmartDashboard.putNumber("ARM Goal Vel", goal.velocity);
  }

  /** Enables the PID control. Resets the controller. */
  @Override
  public void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!m_enabled) {
      m_enabled = true;
      m_controller.reset(getMeasurement());
      DataLogManager.log(
          "Arm Enabled - kP="
              + m_controller.getP()
              + " kI="
              + m_controller.getI()
              + " kD="
              + m_controller.getD()
              + " PosGoal="
              + m_controller.getGoal()
              + " CurPos="
              + getMeasurement());
    }
  }

  /** Disables the PID control. Sets output to zero. */
  @Override
  public void disable() {

    // Set goal to current position to minimize movement on re-enable and reset output
    m_enabled = false;
    setGoal(getMeasurement());
    useOutput(0, new State());
    DataLogManager.log("Arm Disabled");
  }

  public TrapezoidProfile.State getGoal() {
    return lastGoal;
  }
  
}
