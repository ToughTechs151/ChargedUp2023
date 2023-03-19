// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft =
      new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax rearLeft =
      new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax frontRight =
      new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax rearRight =
      new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private final MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(frontLeft, frontRight);
  private final MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(frontRight, rearRight);


  private final DifferentialDrive drive =
      new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  // The front-left-side drive encoder
  private final RelativeEncoder frontLeftEncoder = this.frontLeft.getEncoder();

  // The rear-left-side drive encoder
  private final RelativeEncoder rearLeftEncoder = this.rearLeft.getEncoder();

  // The front-right--side drive encoder
  private final RelativeEncoder frontRightEncoder = this.frontRight.getEncoder();

  // The rear-right-side drive encoder
  private final RelativeEncoder rearRightEncoder = this.rearRight.getEncoder();

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          this.gyro.getRotation2d(),
          frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

  // drive constants
  /** The scaling factor between the joystick value and the speed controller. */
  private double speedMultiplier = 0.5;

  /** The scale factor for normal mode. */
  private static final double NORMAL = 1.0;

  /** The scale factor for crawl mode. */
  private static final double CRAWL = 0.3;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    this.frontLeft.restoreFactoryDefaults();
    this.frontRight.restoreFactoryDefaults();
    this.rearLeft.restoreFactoryDefaults();
    this.rearRight.restoreFactoryDefaults();

    this.frontLeft.setIdleMode(IdleMode.kCoast);
    this.frontRight.setIdleMode(IdleMode.kCoast);
    this.rearLeft.setIdleMode(IdleMode.kCoast);
    this.rearRight.setIdleMode(IdleMode.kCoast);

    //rearLeft.follow(frontLeft);
    //rearRight.follow(frontRight);

    // Sets the distance per pulse for the encoders
    this.frontLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.rearLeftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.frontRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.rearRightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.frontLeftEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOVITY_CONVERSION);
    this.rearLeftEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOVITY_CONVERSION);
    this.frontRightEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOVITY_CONVERSION);
    this.rearRightEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOVITY_CONVERSION);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotorControllerGroup.setInverted(true);
    SmartDashboard.putData(this.drive);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.odometry.update(this.gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param leftSpeed The left joystick controller spped -1 to 1
   * @param rightSpeed The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean isCrawl) {
    speedMultiplier = isCrawl ? CRAWL : NORMAL;
    drive.tankDrive(leftSpeed * speedMultiplier, rightSpeed * speedMultiplier, true);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
    //frontLeft.setVoltage(leftVolts);
    //frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(this.gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xaxisSpeed Speed of the robot in the x direction (forward/backwards).
   * @param yaxisSpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
 // public void drive(double xaxisSpeed, double yaxisSpeed, double rot, boolean fieldRelative)

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.frontLeftEncoder.setPosition(0);
    this.rearLeftEncoder.setPosition(0);
    this.frontRightEncoder.setPosition(0);
    this.rearRightEncoder.setPosition(0);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        this.frontLeftEncoder.getVelocity(),
        this.frontRightEncoder.getVelocity());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -this.gyro.getRate();
  }
}
