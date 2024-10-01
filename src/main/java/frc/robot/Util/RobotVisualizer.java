// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/** Add your docs here. */
public class RobotVisualizer {
  private final ArmSubsystem armSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH);


  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);

  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the arm

  MechanismRoot2d root = panel.getRoot("armSim", 0.4, 0.1); 
  MechanismLigament2d m_arm = root.append(new MechanismLigament2d("arm", 0.3, 90));
  
  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the intake
  MechanismLigament2d m_spinner = m_arm.append(new MechanismLigament2d("spinner", 0.2, 270, 12, new Color8Bit(Color.kPurple)));


  public RobotVisualizer(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    SmartDashboard.putNumber("Robot Visualizer/Pretend Intake Angle", 90);
    SmartDashboard.putNumber("Robot Visualizer/Pretend Arm Angle", 90);
  }

  public void update() {
    //TO DO: When intake subsystem is finished, replace this method to return actual intake arm angle
    double intakeArmAngle = SmartDashboard.getNumber("Robot Visualizer/Pretend Intake Angle", 90);
    //TO DO: When arm subsystem is finished, replace this method to return actual arm angle
    double armAngle = SmartDashboard.getNumber("Robot Visualizer/Pretend Arm Angle", 90);

    //TO DO: Update arm angles in stick figures; 90 degrees is straight down. 180 degrees is perpendicular to floor and above intake
    m_arm.setAngle(armAngle);


    //TO DO: Update intake arm angles in stick figures; 0 degree is deployed n, 92 degrees is retracted position
    m_spinner.setAngle(-intakeArmAngle-90);


    SmartDashboard.putData("Robot Visualizer/panel", panel);
  }
}
