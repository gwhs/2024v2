// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class ReactionSubsystem extends SubsystemBase {
  private TalonFX m_reactionArm;
  private final double REACTION_BAR_PID_KP = 0.09;
  private final double REACTION_BAR_PID_KI = 0;
  private final double REACTION_BAR_PID_KD = 0;
  public PIDController PIDcontroller = new PIDController(REACTION_BAR_PID_KP, REACTION_BAR_PID_KI, REACTION_BAR_PID_KD); 

  /** Creates a new ReactionSubsystem. */
  public ReactionSubsystem(int armID, String canbus) {
    m_reactionArm = new TalonFX(armID, canbus);
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    m_reactionArm.getConfigurator().apply(motorOutput);
    PIDcontroller.setTolerance(Constants.ReactionConstants.tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double PIDvalue = PIDcontroller.calculate(getPos());
    if (PIDvalue > 1)
      PIDvalue = 1;

    if (PIDvalue < -1)
      PIDvalue = -1;

    m_reactionArm.set(PIDvalue);
  }

  public void spinForward()
  {
    PIDcontroller.setSetpoint(-2.5);
  }
  public void spinBackward()
  {
    PIDcontroller.setSetpoint(-0.21);
  }
  public void stop()
  {
    m_reactionArm.set(0);
  }

  public double getPos()
  {
    return m_reactionArm.getRotorPosition().getValue();
  }

  
}
