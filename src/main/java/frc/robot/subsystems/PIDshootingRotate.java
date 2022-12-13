// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimeLight;

public class PIDshootingRotate extends SubsystemBase{
  private static CANSparkMax m_Rotator;
  private static CANSparkMax m_angleRotator;
  public static RelativeEncoder m_RotateEncoder;
  public static RelativeEncoder m_AngleRotateEncoder;
  //public static DigitalInput leftRotateTurretLimitSwitch;
  //public static DigitalInput rightRotateTurretLimitSwitch;
  private Timer t;

  private static double kP, kI, kD, P, I, D, error, errorSum, errorRate, lastTimeStamp, iLimit, lastError;
  public static double tolerance; 
  private static double leftTurretRange, rightTurretRange;
  private static boolean automate;

  public PIDshootingRotate() {
    m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
    m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);
    m_RotateEncoder = m_Rotator.getEncoder();
    m_AngleRotateEncoder = m_angleRotator.getEncoder();
    //leftRotateTurretLimitSwitch = new DigitalInput(AutoConstants.leftRotateTurretLimitSwitch);
    //rightRotateTurretLimitSwitch = new DigitalInput(AutoConstants.rightRotateTurretLimitSwitch);
    t = new Timer();
    automate = false;

    kP = 0.18;
    kI = 0.025;
    kD = 0.3; 
    tolerance = 0.5; 
    iLimit = 2.0;

    /* 
     * Turret 110 degrees left and 75 degrees right. If it goes any further the motor is going to 
     * lose contact with the teeth of the turret gear. There is a chance that if the motor gets there it cant recover so I will
     * put the limit at 105 degrees left and 72 degrees right. The sensor we are using is an encoder
     * so degrees have to be converted to rotations. Motor gear ratio: 100/1, Comes out with a 10 gear, turret is 140 teeth. 
     * 1/100 * 10/140 * 360 = 0.257 degrees per rotation. 
    */
    leftTurretRange = (105/0.257);
    rightTurretRange = -1 * (72/0.257);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   error = LimeLight.limelightTrackingX();

   if(automate == true){
    turnDegrees();
   }
   else{
    runXMotor(RobotContainer.getRightStickXAxis());
   } 
  }

  public double calculate(){
    //integral
    if(Math.abs(error) < iLimit){
      errorSum += error;
    }

    //derivative
    double deltaT = t.getFPGATimestamp() - lastTimeStamp;
    errorRate = (error - lastError) / deltaT;
    lastError = error;
    lastTimeStamp = t.getFPGATimestamp();

    P = kP * error;
    I = kI * errorSum;
    D = kD * errorRate;

    double outputSpeed = P + I + D;

    return outputSpeed;
  }

  public void startNewPID(){
    switchToAuto();
    m_RotateEncoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    t.start();
    lastTimeStamp = t.getFPGATimestamp();
    lastError = LimeLight.limelightTrackingX();
  }

  public void turnDegrees(){
    m_Rotator.set(this.calculate());
  }

  public void switchToAuto(){
    automate = true;
  }

  public void switchToJoystick(){
    automate = false;
  }

  public void resetHood(){
    final double speedY = 0.35;
    while(RobotContainer.hoodLimitSwitch.get() == true){
      System.out.println("fj");
      m_angleRotator.set(-1*speedY);
    }
    m_angleRotator.set(0);
    (this.m_AngleRotateEncoder).setPosition(0);

  }

  public double desiredAngleYToRotations(double desiredAngleY){
    //with (1/1000 * 34/100.8) gear ratio -> .1214 degrees per rotation
    double rotations = desiredAngleY/.1214;
    return rotations;
  }

  public void runXMotor(double rightStickXaxis){
    double speed = 0.4;
    double motorDrive = rightStickXaxis*speed;
    m_Rotator.set(motorDrive);
  }

  public void limeLightAutoAdjustX(double offsetX){
    while(Math.abs(offsetX) > 4 ){
      final double speedX = 0.4;
      if(offsetX > 0 ){
        m_Rotator.set(-1*speedX);
      }
      if(offsetX < 0){
        m_Rotator.set(1*speedX);
      }
    }
    m_Rotator.set(0);
  }

  public void adjustHood(){
    //no PID loop
    double angle = LimeLight.calcHoodAngle();
    final double speed = 0.35;
    while (Math.abs(this.m_AngleRotateEncoder.getPosition()-angle) > 10){
      if(this.m_AngleRotateEncoder.getPosition() > angle){
        m_angleRotator.set(-1*speed);
      } else{
        m_angleRotator.set(speed);
      }
    }
    m_angleRotator.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
