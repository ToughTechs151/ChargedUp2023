// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft =
      new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax rearLeft =
      new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax frontRight =
      new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax rearRight =
      new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  private final MecanumDrive drive =
      new MecanumDrive(this.frontLeft, this.rearLeft, this.frontRight, this.rearRight);

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
  MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(
          DriveConstants.kDriveKinematics,
          this.gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

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

    // Sets the distance per pulse for the encoders
    this.frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    this.rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    this.frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    this.rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    this.frontRight.setInverted(true);
    this.rearRight.setInverted(true);
    SmartDashboard.putData(this.drive);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.odometry.update(this.gyro.getRotation2d(), getCurrentWheelDistances());
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
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(this.gyro.getRotation2d(), getCurrentWheelDistances(), pose);
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
      this.drive.driveCartesian(xaxisSpeed, yaxisSpeed, rot, this.gyro.getRotation2d());
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
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        this.frontLeftEncoder.getVelocity(),
        this.rearLeftEncoder.getVelocity(),
        this.frontRightEncoder.getVelocity(),
        this.rearRightEncoder.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        this.frontLeftEncoder.getPosition(),
        this.rearLeftEncoder.getPosition(),
        this.frontRightEncoder.getPosition(),
        this.rearRightEncoder.getPosition());
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
