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
  private Timer xtimer, ytimer;
  private static double currentHoodAngle, desiredHoodAngle;

  private static double xkP, xkI, xkD, xP, xI, xD, xerror, xerrorSum, xerrorRate, xlastTimeStamp, xiLimit, xlastError;
  public static double xtolerance; 
  private static double xleftTurretRange, xrightTurretRange;
  private static boolean xautomate;

  private static double ykP, ykI, ykD, yP, yI, yD, yerrorSum, yerrorRate, ylastTimeStamp, yiLimit, ylastError, lastHoodPosition;
  public static double ytolerance, yerror;
  private static double yhoodRange;
  private static boolean yautomate;


  public PIDshootingRotate() {
    m_Rotator = new CANSparkMax(AutoConstants.shootRotate, MotorType.kBrushless);
    m_angleRotator = new CANSparkMax(AutoConstants.shootAngleRotate, MotorType.kBrushless);
    m_RotateEncoder = m_Rotator.getEncoder();
    m_AngleRotateEncoder = m_angleRotator.getEncoder();
    //leftRotateTurretLimitSwitch = new DigitalInput(AutoConstants.leftRotateTurretLimitSwitch);
    //rightRotateTurretLimitSwitch = new DigitalInput(AutoConstants.rightRotateTurretLimitSwitch);
    xtimer = new Timer();
    xautomate = false;
    xkP = 0.18;
    xkI = 0.025;
    xkD = 0.3; 
    xtolerance = 0.5; 
    xiLimit = 2.0;
    /* 
     * Turret 110 degrees left and 75 degrees right. If it goes any further the motor is going to 
     * lose contact with the teeth of the turret gear. There is a chance that if the motor gets there it cant recover so I will
     * put the limit at 105 degrees left and 72 degrees right. The sensor we are using is an encoder
     * so degrees have to be converted to rotations. Motor gear ratio: 100/1, Comes out with a 10 gear, turret is 140 teeth. 
     * 1/100 * 10/140 * 360 = 0.257 degrees per rotation. 
    */
    xleftTurretRange = (105/0.257);
    xrightTurretRange = -1 * (72/0.257);

    ytimer = new Timer();
    yautomate = false;
    ykP = 0.5;
    ykI = 0.3;
    ykD = 0.1;
    ytolerance = 0.5;
    yiLimit = 2.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   xerror = LimeLight.limelightTrackingX();

   desiredHoodAngle = LimeLight.calcHoodAngle();
   currentHoodAngle = m_AngleRotateEncoder.getPosition()* 0.1214;
   yerror = desiredHoodAngle - currentHoodAngle;

   SmartDashboard.putNumber("desired hood angle", desiredHoodAngle);
   SmartDashboard.putNumber("current hood angle", currentHoodAngle);
   SmartDashboard.putNumber("yerror", yerror);



   if(xautomate == true){
    turnDegreesX();
   }
   if(yautomate == true){
    turnDegreesY();
   }
   else{
    runXMotor(RobotContainer.getRightStickXAxis());
   } 
  }

  public double calculateY(){
    //integral
    if(Math.abs(yerror) < yiLimit){
      yerrorSum += yerror;
    }

    //derivative
    double deltaT = ytimer.getFPGATimestamp() - ylastTimeStamp;
    yerrorRate = (yerror - ylastError) / deltaT;
    ylastError = yerror;
    ylastTimeStamp = ytimer.getFPGATimestamp();

    yP = ykP * yerror;
    yI = ykI * yerrorSum;
    yD = ykD * yerrorRate;

    double outputSpeed_y = yP + yI + yD;

    return outputSpeed_y;
  }

  public void startNewYPID(){
    switchYToAuto();
    m_RotateEncoder.setPosition(0);
    yerrorSum = 0;
    ylastError = 0;
    ytimer.start();
    ylastTimeStamp = ytimer.getFPGATimestamp();
    ylastError = LimeLight.calcHoodAngle() - currentHoodAngle;
  }

  public void turnDegreesY(){
    m_angleRotator.set(this.calculateY());
  }

  public void switchYToAuto(){
    yautomate = true;
  }

  public void endYPID(){
    yautomate = false;
    currentHoodAngle = desiredHoodAngle - yerror;
  } 

  public void startNewXPID(){
    switchXToAuto();
    m_RotateEncoder.setPosition(0);
    xerrorSum = 0;
    xlastError = 0;
    xtimer.start();
    xlastTimeStamp = xtimer.getFPGATimestamp();
    xlastError = LimeLight.limelightTrackingX();
  }

  public double calculateX(){
    //integral
    if(Math.abs(xerror) < xiLimit){
      xerrorSum += xerror;
    }

    //derivative
    double deltaT = xtimer.getFPGATimestamp() - xlastTimeStamp;
    xerrorRate = (xerror - xlastError) / deltaT;
    xlastError = xerror;
    xlastTimeStamp = xtimer.getFPGATimestamp();

    xP = xkP * xerror;
    xI = xkI * xerrorSum;
    xD = xkD * xerrorRate;

    double outputSpeed_x = xP + xI + xD;

    return outputSpeed_x;
  }

  public void turnDegreesX(){
    m_Rotator.set(this.calculateX());
  }

  public void switchXToAuto(){
    xautomate = true;
  }

  public void switchXToJoystick(){
    xautomate = false;
  }

  public static void resetHood(){
    final double speedY = 0.35;
    while(RobotContainer.hoodLimitSwitch.get() == true){
      System.out.println("HELLO");
      m_angleRotator.set(-1*speedY);
    }
    m_angleRotator.set(0);
    m_AngleRotateEncoder.setPosition(0);
    currentHoodAngle = 0;

  }

  public double desiredAngleYToRotations(double desiredAngleY){
    //with (1/1000 * 34/100.8) gear ratio -> .1214 degrees per rotation
    double rotations = desiredAngleY/.1214;
    return rotations;
  }

  public double encoderToAngle(double position){
    return position * 0.1214;
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
