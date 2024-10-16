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
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH*3);


  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);

  // Code for the stick figure of each subsystems
  // Code for superstructure 
  MechanismRoot2d root = panel.getRoot("superStructure", 0.44, 0); 
  MechanismLigament2d m_superStructure = root.append(new MechanismLigament2d("superStructureL", 1.03, 90));
  
  // Code for arm
  MechanismLigament2d m_arm = m_superStructure.append(new MechanismLigament2d("arm", 0.38, -90));

  //code for pizzaBox
  MechanismLigament2d m_pizzaBox = m_arm.append(new MechanismLigament2d("pizzaBox", 0.38, -90));

  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the intake
  // Example of attaching to roots/ligaments: MechanismLigament2d m_spinner = m_arm.append(new MechanismLigament2d("spinner", 0.2, 270, 12, new Color8Bit(Color.kPurple)));
  
  // code for intakes
  MechanismRoot2d root1 = panel.getRoot("intakeJoint", 0.57, 0.15);
  MechanismLigament2d m_intakeJoint = root1.append(new MechanismLigament2d("intakeArm1", 0.34, 90));

  MechanismLigament2d m_intakeJoint2 = m_intakeJoint.append(new MechanismLigament2d("intakeArm2", 0.23, -135)); 

  //code for climber
  MechanismRoot2d root2 = panel.getRoot("climber", 0.57, 0.15);
  MechanismLigament2d m_climber = root2.append(new MechanismLigament2d("climber", 0.34, 90));
  
  // code for reaction bar
  MechanismRoot2d root3 = panel.getRoot("reactionBar", 0, 0.3);
  MechanismLigament2d m_reactionBar = root3.append(new MechanismLigament2d("reactionBar", 0.47, -45));

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
    m_arm.setAngle(-armAngle);

    //TO DO: Update intake arm angles in stick figures; 0 degree is deployed n, 92 degrees is retracted position
    m_intakeJoint.setAngle(intakeArmAngle);


    SmartDashboard.putData("Robot Visualizer/panel", panel);
  }
}
