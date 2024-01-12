// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Constants Utility class");
  }

  public static final class DriveConstants {

    private DriveConstants() {
      throw new IllegalStateException("DriveConstants Utility class");
    }

    public static final int FRONT_LEFT_MOTOR_PORT = 4;
    public static final int REAR_LEFT_MOTOR_PORT = 1;
    public static final int FRONT_RIGHT_MOTOR_PORT = 2;
    public static final int REAR_RIGHT_MOTOR_PORT = 3;

    // public static final List<Integer> FRONT_LEFT_ENCODER_PORTS = List.of(0, 1)
    // public static final List<Integer> REAR_LEFT_ENCODER_PORTS = List.of(2, 3)
    // public static final List<Integer> FRONT_RIGHT_ENCODER_PORTS = List.of(4, 5)
    // public static final List<Integer> REAR_RIGHT_ENCODER_PORTS = List.of(6, 7)

    // public static final boolean FRONT_LEFT_ENCODER_REVERSED = false
    // public static final boolean REAR_LEFT_ENCODER_REVERSED = true
    // public static final boolean FRONT_RIGHT_ENCODER_REVERSED = false
    // public static final boolean REAR_RIGHT_ENCODER_REVERSED = true

    public static final double TRACK_WIDTH = 0.5;
    // public static final List<Integer> REAR_LEFT_ENCODER_PORTS = List.of(2, 3)
    // public static final List<Integer> FRONT_RIGHT_ENCODER_PORTS = List.of(4, 5)
    // public static final List<Integer> REAR_RIGHT_ENCODER_PORTS = List.of(6, 7)

    // public static final boolean FRONT_LEFT_ENCODER_REVERSED = false
    // public static final boolean REAR_LEFT_ENCODER_REVERSED = true
    // public static final boolean FRONT_RIGHT_ENCODER_REVERSED = false
    // public static final boolean REAR_RIGHT_ENCODER_REVERSED = true

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final double kTrackwidthMeters = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double GEAR_RATIO = 10.71;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double ENCODER_DISTANCE_PER_REVOLUTION =
        (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO;
    public static final double ENCODER_VELOCITY_CONVERSION = ENCODER_DISTANCE_PER_REVOLUTION / 60;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.13923;
    public static final double kvVoltSecondsPerMeter = 2.244;
    public static final double kaVoltSecondsSquaredPerMeter = 0.23774;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.12795;
  }

  public static final class OIconstants {

    private OIconstants() {
      throw new IllegalStateException("OIConstants Utility class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
  }

  public static final class AutoConstants {

    private AutoConstants() {
      throw new IllegalStateException("AutoConstants Utility class");
    }

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static final class ArmConstants {
    public static final int kMotorPort = 5;

    // Constants tunable through preferences
    public static final String ARM_KP_KEY = "ArmKP"; // The P gain for the PID controller
    public static final String ARM_KS_KEY = "ArmKS"; // Static motor gain
    public static final String ARM_KG_KEY = "ArmKG"; // Gravity gain
    public static final String ARM_KV_KEY = "ArmKV"; // Velocity gain
    public static final String ARM_VELOCITY_MAX_KEY = "ArmVelocityMax";
    public static final String ARM_ACCELERATION_MAX_KEY = "ArmAccelerationMax";

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    public static final double kMaxVelocityRadPerSecond = Units.degreesToRadians(90);
    public static final double kMaxAccelerationRadPerSecSquared = Units.degreesToRadians(56.25);

    public static final int[] kEncoderPorts = new int[] {4, 5};
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    // ARM conversions
    public static final double ARM_GEAR_RATIO = 1.0d / 64;
    public static final double ARM_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI * ARM_GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = ARM_RAD_PER_ENCODER_ROTATION / 60;

    // These positions are all in the PID controller reference: units are radians, positive is up
    // and horizontal is 0.0.  Values are displayed to the operator in degrees.

    // The offset of the arm from horizontal to its neutral start position
    public static final double ARM_OFFSET_RADS = Units.degreesToRadians(110.0);

    public static final double ARM_UP_POSITION_RADS = Units.degreesToRadians(110.0);
    public static final double ARM_SCORE_LOW_POSITION_RADS = Units.degreesToRadians(45.0);
    public static final double ARM_SCORE_HIGH_POSITION_RADS = Units.degreesToRadians(55.0);
    public static final double ARM_DOWN_POSITION_RADS = Units.degreesToRadians(-5.0);
    public static final double ARM_RED_ZONE_RADS = Units.degreesToRadians(25.0);
    public static final double ARM_BOTTOM_POSITION_RADS = Units.degreesToRadians(-5.0);
  }

  // Camera ID
  public static final int CAMERA_0 = 0;
  public static final int CAMERA_1 = 1;

  public static final int CLAW_MOTOR = 6;
  public static final int ARM_SOLENOID_KFORWARD = 0;
  public static final int ARM_SOLENOID_KREVERSE = 1;
  public static final int CLAW_SOLENOID_KFORWARD = 2;
  public static final int CLAW_SOLENOID_KREVERSE = 3;

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set to true to log loop timing data. To false to disable.
  public static final boolean LOOP_TIMING_LOG = true;

  // RevRobotics Blinkin
  public static final int BLIKIN_SPARK_PORT = 0;
  public static final double BLINKIN_RED = 0.61;
  public static final double BLINKIN_DARK_GREEN = 0.75;
  public static final double ARM_UP_POSITION = 0.0;
  public static final double ARM_SCORE_LOW_POSITION = 17.0;
  public static final double ARM_SCORE_HIGH_POSITION = 10.0;
  public static final double ARM_DOWN_POSITION = 21.0;
}
