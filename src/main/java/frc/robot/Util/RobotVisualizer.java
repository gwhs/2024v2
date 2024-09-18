// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class RobotVisualizer {
  private final ArmSubsystem armSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH);

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);


  
  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the arm



  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the intake



  public RobotVisualizer(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    SmartDashboard.putNumber("Robot Visualizer/Pretend Intake Angle", 0);
    SmartDashboard.putNumber("Robot Visualizer/Pretend Arm Angle", 90);
  }

  public void update() {
    double intakeArmAngle = getIntakeArmAngle();
    double armAngle = getArmAngle();

    //TO DO: Update arm angles in stick figures; 90 degrees is straight down. 180 degrees is perpendicular to floor and above intake



    //TO DO: Update intake arm angles in stick figures; 0 degree is deployed position, 92 degrees is retracted position



    SmartDashboard.putData("Robot Visualizer/panel", panel);
  }

  private double getIntakeArmAngle() {
    //TO DO: When intake subsystem is finished, replace this method to return actual intake arm angle
    return SmartDashboard.getNumber("Robot Visualizer/Pretend Intake Angle", 0);
  }

  private double getArmAngle() {
    //TO DO: When arm subsystem is finished, replace this method to return actual arm angle
    return SmartDashboard.getNumber("Robot Visualizer/Pretend Arm Angle", 90);
  }
}
