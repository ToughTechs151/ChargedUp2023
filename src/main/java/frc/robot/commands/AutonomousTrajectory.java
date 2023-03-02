// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTrajectory extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous Drive based on distance. This will drive out for a
     * specified distance,
     * turn around and drive back.
     *
     * @param drivetrain The drivetrain subsystem on which this command will run
     * @return
     */
    public AutonomousTrajectory(RobotContainer robotContainer, Trajectory trajectory) {
        DriveSubsystem m_robotDrive = robotContainer.getDriveSubsystem();

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

    

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kFeedforward,
                DriveConstants.kDriveKinematics,

                // Position contollers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

                // Needed for normalizing wheel speeds
                AutoConstants.kMaxSpeedMetersPerSecond,

                // Velocity PID's
                new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
                new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
                new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
                new PIDController(DriveConstants.kPRearRightVel, 0, 0),
                m_robotDrive::getCurrentWheelSpeeds,
                m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        addCommands(mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)));
    }
}
