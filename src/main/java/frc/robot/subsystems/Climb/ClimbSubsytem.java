// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import java.util.Map;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ClimbSubsytem extends SubsystemBase {
  private ClimbIO climbIO;
  private Constraints constraints = new Constraints(ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCELERATION);

  private ProfiledPIDController leftpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP,
      ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);
  private ProfiledPIDController rightpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP,
      ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);

  public ClimbSubsytem() {

    if (RobotBase.isSimulation()) {
      climbIO = new ClimbIOSim();
    }
    else {
      climbIO = new ClimbIOReal();
      
    }

    leftpidController.setGoal(climbIO.getLeftMotorPosition());
    rightpidController.setGoal(climbIO.getRightMotorPosition());

    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout climbCommandsLayout = tab.getLayout("TestingCommands", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN"));

    climbCommandsLayout.add(motorUp());
    climbCommandsLayout.add(motorDown());

    SmartDashboard.putData("LeftPIDController", leftpidController);
    SmartDashboard.putData("RightPIDController", rightpidController);

  }

  @Override
  public void periodic() {
    double rightpidOutput = rightpidController.calculate(climbIO.getRightMotorPosition());
    double leftpidOutput = leftpidController.calculate(climbIO.getLeftMotorPosition());

    rightpidOutput = MathUtil.clamp(rightpidOutput, -1, 1);
    leftpidOutput = MathUtil.clamp(leftpidOutput, -1, 1);

//start higher
    // if limit goes up and limit switch activates, stop motor

    climbIO.setLeftMotorSpeed(leftpidOutput);
    climbIO.setRightMotorSpeed(rightpidOutput);
    climbIO.update();

    NetworkTableInstance.getDefault().getEntry("Climb/Left PID Output").setNumber(leftpidOutput);
    NetworkTableInstance.getDefault().getEntry("Climb/Right PID Output").setNumber(rightpidOutput);
    NetworkTableInstance.getDefault().getEntry("Climb/Left PID Goal").setNumber(leftpidController.getGoal().position);
    NetworkTableInstance.getDefault().getEntry("Climb/Right PID Goal").setNumber(rightpidController.getGoal().position);
    NetworkTableInstance.getDefault().getEntry("Climb/Left motor Position").setNumber(climbIO.getLeftMotorPosition());
    NetworkTableInstance.getDefault().getEntry("Climb/Right motor Position").setNumber(climbIO.getRightMotorPosition());

  }

  public Command motorUp() {
    return this.runOnce(() -> {
      leftpidController.setGoal(ClimbConstants.LEFT_UP_POSITION);
      rightpidController.setGoal(ClimbConstants.RIGHT_UP_POSITION);
    }).withName("Motor Up");
  }

  public Command motorDown() {
    return this.runOnce(() -> {
      leftpidController.setGoal(ClimbConstants.LEFT_DOWN_POSITION);
      rightpidController.setGoal(ClimbConstants.RIGHT_DOWN_POSITION);
    }).withName("Motor Down");
  }
}
