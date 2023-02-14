// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.sim.RobotModel;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private DataLogging datalog;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Initialize the data logging.
    datalog = DataLogging.getInstance();

    // Print our splash screen info.
    Splash.printAllStatusFiles();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    datalog.dataLogRobotContainerInit(m_robotContainer);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // must be at the end of robotPeriodic
    datalog.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (isSimulation() && simModel != null) {
      simModel.reset();
    }

    // Add code for entering disabled mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    datalog.startLoopTime();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    if (m_robotContainer == null) {
      DriverStation.reportError("autonomousInit called with null robotContainer", false);
    } else {
      // Cancel any commands already running.
      CommandScheduler.getInstance().cancelAll();

      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      /*
       * String autoSelected = SmartDashboard.getString("Auto Selector",
       * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
       * = new MyAutoCommand(); break; case "Default Auto": default:
       * autonomousCommand = new ExampleCommand(); break; }
       */

      // schedule the autonomous command (example)
      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    datalog.startLoopTime();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    datalog.startLoopTime();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    teleopInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  // Simple robot plant model for simulation purposes
  RobotModel simModel;

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // Add code to run when the robot is initialized during simulations.
    simModel = new RobotModel(this);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Add code to run repeatedly during simulations.
    if (isSimulation() && simModel != null) {
      simModel.update();
    }
  }

  public RobotContainer getRobotContainer() {
    return m_robotContainer;
  }
}
