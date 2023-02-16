// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ARM_SOLENOID_KFORWARD, Constants.ARM_SOLENOID_KREVERSE);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void back(){
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void forward(){
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    
  }
}
