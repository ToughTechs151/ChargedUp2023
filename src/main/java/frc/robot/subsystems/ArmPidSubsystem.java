// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
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
  private ArmFeedforward feedforward =
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
    m_enabled = false;

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setVoltage(0.0);

    blinkinSpark.set(blinkinVoltage);

    encoder.setPositionConversionFactor(ArmConstants.ARM_RAD_PER_ENCODER_ROTATION);
    encoder.setVelocityConversionFactor(ArmConstants.RPM_TO_RAD_PER_SEC);
    encoder.setPosition(0);

    initPreferences();

    SmartDashboard.putData(m_controller);

    // Dummy values so they are available for telemetry before goal is set
    SmartDashboard.putNumber("ARM Goal Pos", 0);
    SmartDashboard.putNumber("ARM Goal Vel", 0);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    }

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

  public void setGoalPosition(TrapezoidProfile.State goal) {
    lastGoal = goal;
    super.setGoal(goal);
    SmartDashboard.putNumber("ARM Goal Pos", Units.radiansToDegrees(goal.position));
    SmartDashboard.putNumber("ARM Goal Vel", Units.radiansToDegrees(goal.velocity));
  }

  /** Enables the PID control. Resets the controller. */
  @Override
  public void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!m_enabled) {
      loadPreferences();

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

    // Clear the enabled flag and call useOutput to zero the motor command
    m_enabled = false;
    useOutput(0, new State());

    DataLogManager.log(
        "Arm Disabled CurPos="
            + Units.radiansToDegrees(getMeasurement())
            + " CurVel="
            + Units.radiansToDegrees(encoder.getVelocity()));
  }

  public TrapezoidProfile.State getGoal() {
    return lastGoal;
  }

  /**
   * Put tunable values in the Preferences table using default values, if the keys don't already
   * exist.
   */
  private void initPreferences() {

    // Preferences for PID controller
    Preferences.initDouble(Constants.ArmConstants.ARM_KP_KEY, Constants.ArmConstants.kP);

    // Preferences for Trapezoid Profile
    Preferences.initDouble(
        Constants.ArmConstants.ARM_VELOCITY_MAX_KEY,
        Constants.ArmConstants.kMaxVelocityRadPerSecond);
    Preferences.initDouble(
        Constants.ArmConstants.ARM_ACCELERATION_MAX_KEY,
        Constants.ArmConstants.kMaxAccelerationRadPerSecSquared);

    // Preferences for Feedforward
    Preferences.initDouble(Constants.ArmConstants.ARM_KS_KEY, Constants.ArmConstants.kSVolts);
    Preferences.initDouble(Constants.ArmConstants.ARM_KG_KEY, Constants.ArmConstants.kGVolts);
    Preferences.initDouble(
        Constants.ArmConstants.ARM_KV_KEY, Constants.ArmConstants.kVVoltSecondPerRad);
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    m_controller.setP(
        Preferences.getDouble(Constants.ArmConstants.ARM_KP_KEY, Constants.ArmConstants.kP));

    // Read Preferences for Trapezoid Profile and update
    double velocityMax =
        Preferences.getDouble(
            Constants.ArmConstants.ARM_VELOCITY_MAX_KEY,
            Constants.ArmConstants.kMaxVelocityRadPerSecond);
    double accelerationMax =
        Preferences.getDouble(
            Constants.ArmConstants.ARM_ACCELERATION_MAX_KEY,
            Constants.ArmConstants.kMaxAccelerationRadPerSecSquared);
    m_controller.setConstraints(new TrapezoidProfile.Constraints(velocityMax, accelerationMax));

    // Read Preferences for Feedforward and create a new instance
    double staticGain =
        Preferences.getDouble(Constants.ArmConstants.ARM_KS_KEY, Constants.ArmConstants.kSVolts);
    double gravityGain =
        Preferences.getDouble(Constants.ArmConstants.ARM_KG_KEY, Constants.ArmConstants.kGVolts);
    double velocityGain =
        Preferences.getDouble(
            Constants.ArmConstants.ARM_KV_KEY, Constants.ArmConstants.kVVoltSecondPerRad);

    feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain, 0);
  }
}
