// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIconstants;
import frc.robot.commands.ArmDPadDownCommand;
import frc.robot.commands.ArmDPadUpCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRetractCommand;
import frc.robot.commands.ArmScoreHighCommand;
import frc.robot.commands.ArmScoreLowCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.AutonomousTrajectory;
import frc.robot.commands.ClawCloseCommand;
import frc.robot.commands.ClawInCommand;
import frc.robot.commands.ClawOpenCommand;
import frc.robot.commands.ClawOutCommand;
import frc.robot.subsystems.ArmPidSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private PowerDistribution pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);




  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ArmPidSubsystem armPidSubsystem = new ArmPidSubsystem(this);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // The driver's controller
  private CommandXboxController driverController =
      new CommandXboxController(OIconstants.DRIVER_CONTROLLER_PORT);

  private CommandXboxController codriverController =
      new CommandXboxController(Constants.OIconstants.CODRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    this.robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                this.robotDrive.tankDrive(
                    -this.driverController.getLeftY(),
                    -this.driverController.getRightY(),
                    this.driverController.rightBumper().getAsBoolean()),
            this.robotDrive));
    SmartDashboard.putData("ArmSubsystem", armPidSubsystem);
    armPidSubsystem.disable();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    driverController
        .rightBumper()
        .onTrue(new InstantCommand(() -> this.robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> this.robotDrive.setMaxOutput(1)));
    codriverController.leftBumper().onTrue(new ClawOpenCommand(clawSubsystem));
    codriverController.rightBumper().onTrue(new ClawCloseCommand(clawSubsystem));
    codriverController.leftBumper().whileTrue(new ClawInCommand(clawSubsystem));
    codriverController.rightBumper().whileTrue(new ClawOutCommand(clawSubsystem));
    codriverController.a().onTrue(new ArmDownCommand(armPidSubsystem));
    codriverController.b().onTrue(new ArmUpCommand(armPidSubsystem));
    codriverController.x().onTrue(new ArmScoreHighCommand(armPidSubsystem));
    codriverController.y().onTrue(new ArmScoreLowCommand(armPidSubsystem));
    codriverController.povDown().whileTrue(new ArmDPadDownCommand(armPidSubsystem));
    codriverController.povUp().whileTrue(new ArmDPadUpCommand(armPidSubsystem));

    codriverController.leftTrigger().onTrue(new ArmExtendCommand(armSubsystem));
    codriverController.rightTrigger().onTrue(new ArmRetractCommand(armSubsystem));

    //Autonomous Chooser
    chooser.setDefaultOption("Nothing", "Nothing");
    chooser.addOption("Path1", "Path1");
    chooser.addOption("Test", "Test");
    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);
  }

  public String getAutomousString() {
    return chooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* Return null if there is no autonomous command to run. */
    return null;
    //         DriveConstants.kFeedforward,
    //         // Position contollers
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * This method retrieves the instance of the ArmSubsystem
   *
   * @return the ArmSubsystem instance
   */
  public ArmSubsystem getArmSubsystem() {
    return this.armSubsystem;
  }

  public DriveSubsystem getDriveSubsystem() {
    return this.robotDrive;
  }
}
