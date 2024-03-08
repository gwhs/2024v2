// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;
import frc.robot.testcontainers.ArmContainer;
import frc.robot.testcontainers.ClimbContainer;
import frc.robot.testcontainers.DriveContainer;
import frc.robot.testcontainers.IntakeContainer;
import frc.robot.testcontainers.VisionContainer;
import frc.robot.testcontainers.LEDContainer;
import frc.robot.testcontainers.ReactionArmContainer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot  {

  public static final String GAME = "Game"; 
  public static final String INTAKE = "Intake";
  public static final String DRIVE = "Drive";
  public static final String ARM = "Arm";
  public static final String CLIMB = "Climb";
  public static final String VISION = "Vision";
  public static final String LED = "LED";
  public static final String REACTION = "REACTION";
  
  // change this to match the subsystem container you want to use, or GAME for complete robot
  public static final String container = GAME;

  private Command m_autonomousCommand;

  private BaseContainer m_baseContainer;
  private GameRobotContainer gameRobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and use the subsystems needed
    // for the specific robot
    // LimelightHelpers.setStreamMode_PiPSecondary("limelight");

    // Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5800").withSize(4,3).withPosition(5, 0);

    String logfolder = "/home/lvuser";
    Logger.addDataReceiver(new WPILOGWriter(logfolder));
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    
    switch (container){
      case GAME:
        gameRobotContainer = new GameRobotContainer();
        m_baseContainer = gameRobotContainer;
        break;
      case INTAKE:
        m_baseContainer = new IntakeContainer();
        break;
      default:
      case DRIVE:
        m_baseContainer = new DriveContainer();
        break;
      case CLIMB:
        m_baseContainer = new ClimbContainer();
        break;
      case ARM:
        m_baseContainer = new ArmContainer();
        break;
      case VISION:
        m_baseContainer = new VisionContainer();
        break;
      case LED:
        m_baseContainer = new LEDContainer();
        break;
      case REACTION:
        m_baseContainer = new ReactionArmContainer();
        break;
    }  

   
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
    //     LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    // Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5800").withSize(4,3).withPosition(5, 0);
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_baseContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if(gameRobotContainer != null) {
      gameRobotContainer.teleopInitReset().schedule();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {  }
  

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  
}
