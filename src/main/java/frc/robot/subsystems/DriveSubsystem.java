// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax frontLeft = new PWMSparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT);
  private final PWMSparkMax rearLeft = new PWMSparkMax(DriveConstants.REAR_LEFT_MOTOR_PORT);
  private final PWMSparkMax frontRight = new PWMSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT);
  private final PWMSparkMax rearRight = new PWMSparkMax(DriveConstants.REAR_RIGHT_MOTOR_PORT);

  private final MecanumDrive drive =
      new MecanumDrive(this.frontLeft, this.rearLeft, this.frontRight, this.rearRight);

  // The front-left-side drive encoder
  private final Encoder frontLeftEncoder =
      new Encoder(
          DriveConstants.FRONT_LEFT_ENCODER_PORTS[0],
          DriveConstants.FRONT_LEFT_ENCODER_PORTS[1],
          DriveConstants.FRONT_LEFT_ENCODER_REVERSED);

  // The rear-left-side drive encoder
  private final Encoder rearLeftEncoder =
      new Encoder(
          DriveConstants.REAR_LEFT_ENCODER_PORTS[0],
          DriveConstants.REAR_LEFT_ENCODER_PORTS[1],
          DriveConstants.REAR_LEFT_ENCODER_REVERSED);

  // The front-right--side drive encoder
  private final Encoder frontRightEncoder =
      new Encoder(
          DriveConstants.FRONT_RIGHT_ENCODER_PORTS[0],
          DriveConstants.FRONT_RIGHT_ENCODER_PORTS[1],
          DriveConstants.FRONT_RIGHT_ENCODER_REVERSED);

  // The rear-right-side drive encoder
  private final Encoder rearRightEncoder =
      new Encoder(
          DriveConstants.REAR_RIGHT_ENCODER_PORTS[0],
          DriveConstants.REAR_RIGHT_ENCODER_PORTS[1],
          DriveConstants.REAR_RIGHT_ENCODER_REVERSED);

  // The gyro sensor
  private final Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    this.frontLeftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.rearLeftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.frontRightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    this.rearRightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    this.frontRight.setInverted(true);
    this.rearRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(gyro.getRotation2d(), getCurrentWheelDistances());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getCurrentWheelDistances(), pose);
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
  public void drive(double xaxisSpeed, double yaxisSpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      this.drive.driveCartesian(xaxisSpeed, yaxisSpeed, rot, gyro.getRotation2d());
    } else {
      this.drive.driveCartesian(xaxisSpeed, yaxisSpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    this.frontLeft.setVoltage(volts.frontLeftVoltage);
    this.rearLeft.setVoltage(volts.rearLeftVoltage);
    this.frontRight.setVoltage(volts.frontRightVoltage);
    this.rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.frontLeftEncoder.reset();
    this.rearLeftEncoder.reset();
    this.frontRightEncoder.reset();
    this.rearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return this.frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return this.rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return this.frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return this.rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        this.frontLeftEncoder.getRate(),
        this.rearLeftEncoder.getRate(),
        this.frontRightEncoder.getRate(),
        this.rearRightEncoder.getRate());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        this.frontLeftEncoder.getDistance(),
        this.rearLeftEncoder.getDistance(),
        this.frontRightEncoder.getDistance(),
        this.rearRightEncoder.getDistance());
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
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }
}
