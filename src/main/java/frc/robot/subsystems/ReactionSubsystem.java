// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReactionSubsystem extends SubsystemBase {
  private TalonFX m_reactionArm;
  /** Creates a new ReactionSubsystem. */
  public ReactionSubsystem(int armID) {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
