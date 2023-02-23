// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    throw new IllegalStateException("Utility class: Constants");
  }

  public static final class DriveConstants {

    private DriveConstants() {
      throw new IllegalStateException("Utility class: Drive Constants");
    }

    public static final int FRONT_LEFT_MOTOR_PORT = 0;
    public static final int REAR_LEFT_MOTOR_PORT = 1;
    public static final int FRONT_RIGHT_MOTOR_PORT = 2;
    public static final int REAR_RIGHT_MOTOR_PORT = 3;

    public static final int[] FRONT_LEFT_ENCODER_PORTS = new int[] {0, 1};
    public static final int[] REAR_LEFT_ENCODER_PORTS = new int[] {2, 3};
    public static final int[] FRONT_RIGHT_ENCODER_PORTS = new int[] {4, 5};
    public static final int[] REAR_RIGHT_ENCODER_PORTS = new int[] {6, 7};

    public static final boolean FRONT_LEFT_ENCODER_REVERSED = false;
    public static final boolean REAR_LEFT_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGHT_ENCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_ENCODER_REVERSED = true;

    public static final double TRACK_WIDTH = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    public static final int ENCODER_CPR = 1024;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double ENCODER_DISTANCE_PER_PULSE =
        // Assumes the encoders are directly mounted on the wheel shafts
        (WHEEL_DIAMETER_METERS * Math.PI) / ENCODER_CPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double FRONT_LEFT_VEL = 0.5;
    public static final double REAR_LEFT_VEL = 0.5;
    public static final double FRONT_RIGHT_VEL = 0.5;
    public static final double REAR_RIGHT_VEL = 0.5;
  }

  public static final class OIconstants {

    private OIconstants() {
      throw new IllegalStateException("Utility class: OIconstants");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class AutoConstants {

    private AutoConstants() {
      throw new IllegalStateException("Utility class: AutoConstants");
    }

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double KPX_CONTROLLER = 0.5;
    public static final double KPY_CONTROLLER = 0.5;
    public static final double THETA_CONTROLLER = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static final class ArmConstants {

    private ArmConstants() {
      throw new IllegalStateException("Utility class: ArmConstants");
    }

    public static final int MOTOR_PORT = 2;

    public static final double KP = 1;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double KS_VOLTS = 1;
    public static final double KG_VOLTS = 1;
    public static final double KV_VOLT_SECOND_PER_RAD = 0.5;
    public static final double KA_VOLT_SECOND_SQUARED_PER_RAD = 0.1;

    public static final double MAX_VELOCITY_RAD_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 10;

    public static final int[] ENCODER_PORTS = new int[] {4, 5};
    public static final int ENCODER_PPR = 256;
    public static final double ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / ENCODER_PPR;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double ARM_OFF_SET_RADS = 0.5;
  }

  public static final int SOLENOID_KFORWARD = 1;
  public static final int SOLENOID_KREVERSE = 2;

  // XBox Controller
  public static final int DRIVER_XBOX_CONTROLLER_PORT = 0;
  public static final int CODRIVER_XBOX_CONTROLLER_PORT = 0;

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set to true to log loop timing data. To false to disable.
  public static final boolean LOOP_TIMING_LOG = true;
}
