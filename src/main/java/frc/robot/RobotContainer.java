// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.PIDshootingRotate;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //RR 1/11/2022
  public static final XboxController m_driverController = new XboxController(3);
  public static final XboxController m_driverController2 = new XboxController(1);//change
  public static final PS4Controller m_controller = new PS4Controller(3);

  //INSTANTIATES ALL SUBSYSTEMS
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final BallIntake m_BallIntake = new BallIntake();
  private final Climbing m_Climbing = new Climbing();
  private final Elevator m_Elevator = new Elevator();
  private final RobotDrive m_RobotDrive = new RobotDrive();
  private final Shooting m_Shooting = new Shooting();
  private final PIDshootingRotate m_PIDshootingRotate = new PIDshootingRotate();
  
  //INSTANTIATES ALL COMMANDS
  private final ExampleCommand m_exampleCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoCommand m_AutoCommand = new AutoCommand(m_RobotDrive, m_BallIntake, m_Shooting, m_PIDshootingRotate, m_Elevator, .5);
  private final BallIntakeCommand m_BallIntakeCommand = new BallIntakeCommand(m_BallIntake);
  private final BallShootTopCommand m_BallShootTopCommand = new BallShootTopCommand(m_Shooting);
  private final ClimbingUpCommand m_ClimbingUpCommand = new ClimbingUpCommand(m_Climbing);
  private final ClimbingDownCommand m_ClimbingDownCommand = new ClimbingDownCommand(m_Climbing);
  private final DriveCommand m_DriveCommand = new DriveCommand(m_RobotDrive);
  private final ElevatorMoveBottomCommand m_ElevatorMoveBottomCommand = new ElevatorMoveBottomCommand(m_Elevator);
  private final ElevatorMoveTopCommand m_ElevatorMoveTopCommand = new ElevatorMoveTopCommand(m_Elevator);
  private final BallOutCommand m_BallOutCommand = new BallOutCommand(m_Elevator);
  private final LimeLightAutoAdjustCommand m_LimeLightAutoAdjustCommand = new LimeLightAutoAdjustCommand(m_PIDshootingRotate);
  private final LimeLightAutoAdjustY m_LimeLightAutoAdjustY = new LimeLightAutoAdjustY(m_PIDshootingRotate);
  private final ResetHoodCommand m_ResetHoodCommand = new ResetHoodCommand(m_PIDshootingRotate);


  private static boolean adjustRotateOn = true;
  public static double startingAngle = 0.0;
  public static Encoder leftEncoder = new Encoder(0,1);
  public static Encoder rightEncoder = new Encoder(2, 3);
  //public static Encoder limelightRotateEncoder = new Encoder(4, 5);

  private static AHRS m_gyro = new AHRS(SPI.Port.kMXP); //HC - 01/13/22
  private static PIDController turnController = new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD);
  public static double distanceError = 0;
  // private static DigitalInput ballLimitSwitch = new DigitalInput(AutoConstants.ballLimitSwitchPort);
  public static DigitalInput hoodLimitSwitch = new DigitalInput(6/*AutoConstants.hoodLimitSwitchPort*/);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /**
  * The RobotContainer's constructor.
  * This is where the bulk of the robot should be declared.
  * eg: public RobotContainer() {
  *     m_exampleSubsystem = new ExampleSubsystem();
  *     m_chooser.setDefaultOption("Default Option", new ExampleCommand());
  *     m_chooser.addOption("Option 2", new Command2());
  *     SmartDashboard.putData("Auto mode", m_chooser);
  * }
   */
  public RobotContainer() {
    // Configure the button bindings
    m_DriveCommand.execute();
    configureButtonBindings();
    //(ShootingRotate.m_AngleRotateEncoder).setPosition(0);
    (RobotDrive.m_RLencoder).setPosition(0);
    (RobotDrive.m_RRencoder).setPosition(0);
    (RobotDrive.m_FLencoder).setPosition(0);
    (RobotDrive.m_FRencoder).setPosition(0);
    startingAngle = LimeLight.limelightTrackingX();
    m_gyro.calibrate();

  }

  public double additionalX(){
    double current_angle = startingAngle + m_gyro.getAngle();
    double xVelocity = Math.cos(current_angle)*m_RobotDrive.getSpeed();
    return xVelocity * .02; //change - idk how to do this
  }

  public double additionalY(){
    double current_angle = startingAngle + m_gyro.getAngle();
    double yVelocity = Math.sin(current_angle)*m_RobotDrive.getSpeed();
    return yVelocity * .02; //change - idk how to do this
  }


  public static boolean getHoodLimitSwitch(){
    return hoodLimitSwitch.get();
  }


  public static double getLeftStickY(){
    double axis = m_driverController.getRawAxis(0);
    if(Math.abs(axis) < 0.02)
    {
      axis = 0;
    }
    return axis;
  }

  public static double getLeftStickX(){
    double axis=m_driverController.getRawAxis(1);
    if(axis<0.02&&axis>-0.02)
    {
      axis=0;
    }
    return axis;
  }

  public static double getRightStickXAxis(){
    double axis=m_driverController.getRawAxis(4);
    if(axis<0.02&&axis>-0.02)
    {
      axis=0;
    }
    SmartDashboard.putNumber("Right Stick, X axis", axis);
    return axis;
  }
  
  public static double  getRightStickYAxis(){
    double axis=m_driverController.getRawAxis(5);
    if(axis<0.02&&axis>-0.02)
    {
      axis=0;
    }
    SmartDashboard.putNumber("Right Stick, Y axis", axis);
    return axis;
  }


  public static boolean leftTriggerAxis(){
    double axis = m_driverController.getRawAxis(2);
    return Math.abs(axis)>0.1;
  }
  public static boolean rightTriggerAxis(){
    double axis = m_driverController.getRawAxis(3);
    return Math.abs(axis)>0.1;
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileHeld(m_ClimbingUpCommand);
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileHeld(m_ClimbingDownCommand);
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileHeld(m_BallShootTopCommand); //og : m_BallIntakeCommand
    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileHeld(m_ElevatorMoveTopCommand);
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileHeld(m_BallOutCommand);
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileHeld(m_LimeLightAutoAdjustCommand);

    new JoystickButton(m_driverController2, XboxController.Button.kA.value).whileHeld(m_LimeLightAutoAdjustY);
    new JoystickButton(m_driverController2, XboxController.Button.kY.value).whileHeld(m_ResetHoodCommand);

  }


  public static XboxController getDriveJoystick(){
    return m_driverController;
    
  }

  /**
   * Called in the ShootingRotate subsystem to pass in the boolean adjustRotateOn. - HC
   ?????????
   */
  public static boolean getRotateStatus(){
    return adjustRotateOn;
  }

  public static double getDistance() { 
    leftEncoder.reset();
    rightEncoder.reset();
    return (leftEncoder.getDistance() + rightEncoder.getDistance())/2;
  }

  /**
   * Uses the asymmetric sigmoid w/ quadratic weighting or whatever :/
   * @return hood angle and speed
   */
  
  public static void updateError(double error){
    distanceError += error;
  }

/** HC - 01/12/2022
 * Pseudocode from https://frc-pdr.readthedocs.io/en/latest/control/gyro.html
 * function rotateToAngle(targetAngle):
    error = targetAngle - gyroAngle # check out wpilib documentation for getting the angle from the gyro
    if error > threshold
        this.rotation =  error*kP
        return False
    else:
        this.rotation = 0
        return True
 * 
 */
  // public static boolean rotateToAngle(double targetAngle){
  //   //threshold is subject to change, but represents the accpetable margin of error
  //   double threshold = 5;
  //   double error = targetAngle - m_gyro.getAngle();
  //   if(error > threshold){
  //      /**
  //       * Add code to adjust motor so that the error is reduced
  //       */
      
  //      return false;
  //   } else {
  //     return true;
  //   }
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoCommand;
  }


}
