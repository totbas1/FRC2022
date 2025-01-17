// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class Climbing extends SubsystemBase {
  /** Creates a new Climbing. */
  private static CANSparkMax m_Climbing1 = new CANSparkMax(AutoConstants.climbing, MotorType.kBrushless);
  public Climbing(){}

  public void climbUp(){
    m_Climbing1.set(0.75);
  }
  public void climbDown(){
    m_Climbing1.set(-0.75);
  }

  public void stop(){
    m_Climbing1.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
