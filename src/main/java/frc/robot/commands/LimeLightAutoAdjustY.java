// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.PIDshootingRotate;
import frc.robot.LimeLight;

/** An example command that uses an example subsystem. */
public class LimeLightAutoAdjustY extends CommandBase { 
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PIDshootingRotate m_subsystem;
  boolean isFinished = false;
  boolean inErrorZone = false;
  int count;

  public LimeLightAutoAdjustY(PIDshootingRotate subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.startNewYPID();
  }

  @Override
  public void execute() {
    inErrorZone = Math.abs(m_subsystem.yerror) < m_subsystem.ytolerance;
    m_subsystem.turnDegreesY();
    if(inErrorZone){
      count++;
      if(count >= 3){
        isFinished = true;
      }
    }
    else{
      count = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.endYPID();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

}
