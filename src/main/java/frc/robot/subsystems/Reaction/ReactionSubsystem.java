// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactionSubsystem extends SubsystemBase {
  public TalonFX m_reactionArm = new TalonFX(ReactionConstants.reactionID, ReactionConstants.reactionCAN);
  public PIDController pidController = new PIDController(ReactionConstants.kP, ReactionConstants.kI, ReactionConstants.kD);

  
  /** Creates a new ReactionSubsystem. */
  public ReactionSubsystem() {

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pidController.calculate()
  }
}
