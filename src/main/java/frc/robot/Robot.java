// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.UtilMath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot  {

  public static final String GAME = "Game"; 
  
  // change this to match the subsystem container you want to use, or GAME for complete robot
  public static final String container = GAME;

  private Command m_autonomousCommand;

  private BaseContainer m_baseContainer;

    /*
   * Logging stuff
   */
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final DoublePublisher nt_rioCANUtilization = nt.getDoubleTopic("CAN Utilization/rio").publish();
  private final DoublePublisher nt_canivorCANUtilization = nt.getDoubleTopic("CAN Utilization/Canivore").publish();
  private final DoublePublisher nt_batteryVoltage = nt.getDoubleTopic("Robot/Battery Voltage").publish();
  private final DoublePublisher nt_brownOutBoltage = nt.getDoubleTopic("Robot/Brown Out Voltage").publish();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SignalLogger.setPath("/home/lvuser/log/");
    SignalLogger.enableAutoLogging(false);
    SignalLogger.start();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    UtilMath.overhand.put(1.33, 236d); //S2
    UtilMath.overhand.put(1.79, 240.5);
    UtilMath.overhand.put(2.45, 245d); //A2 
    
    switch (container){
      case GAME:
        m_baseContainer = new GameRobotContainer();
        break;
    }  

    nt_brownOutBoltage.set(RobotController.getBrownoutVoltage());
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

    nt_rioCANUtilization.set(RobotController.getCANStatus().percentBusUtilization);
    nt_canivorCANUtilization.set(CANBus.getStatus("CAN_Network").BusUtilization);

    nt_batteryVoltage.set(RobotController.getBatteryVoltage());

    m_baseContainer.periodic();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link CttreRobotContainer} class. */
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
