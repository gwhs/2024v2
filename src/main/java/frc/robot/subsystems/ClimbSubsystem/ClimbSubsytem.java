// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbSubsystem;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ClimbSubsytem extends SubsystemBase {
  private ClimbIO climbIO;

  public ClimbSubsytem() {

    if (RobotBase.isSimulation()) {
      climbIO = new ClimbIOSim();
    }
    else {
      climbIO = new ClimbIOReal();
      
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout climbCommandsLayout = tab.getLayout("TestingCommands", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN"));

    climbCommandsLayout.add(motorUp());
    climbCommandsLayout.add(motorDown());

  }

  @Override
  public void periodic() {  
    climbIO.update();

    NetworkTableInstance.getDefault().getEntry("Climb/Left motor Position").setNumber(climbIO.getLeftMotorPosition());
    NetworkTableInstance.getDefault().getEntry("Climb/Right motor Position").setNumber(climbIO.getRightMotorPosition());

  }

  public Command motorUp() {
    return this.runOnce(() -> {
      climbIO.setPositionLeft(ClimbConstants.LEFT_UP_POSITION);
      climbIO.setPositionRight(ClimbConstants.RIGHT_UP_POSITION);
    }).withName("Motor Up");
  }

  public Command motorDown() {
    return this.runOnce(() -> {
      climbIO.setPositionLeft(ClimbConstants.LEFT_DOWN_POSITION);
      climbIO.setPositionLeft(ClimbConstants.RIGHT_DOWN_POSITION);
    }).withName("Motor Down");
  }
}
