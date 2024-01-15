// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_lowerIntake;
  private TalonFX m_spinIntake1;
  private TalonFX m_spinIntake2;
  
  // int lowerIntakeId: Id for lowerng motors for the intake
  // int spinIntake1Id: Id for spining first intake motor 
  // int spinIntake2Id: Id for spining second intake motor 
  // String can: String ID of canivore  
  public IntakeSubsystem(int lowerIntakeId, int spinIntake1Id, int spinIntake2Id, String can) {
    // init motor 
    m_lowerIntake = new TalonFX(lowerIntakeId, can); 
    m_spinIntake1 = new TalonFX(spinIntake1Id, can);
    m_spinIntake2 = new TalonFX(spinIntake2Id, can);
  }

  //Sets the angle for the intake motor
  //TODO: Limit the maximum angle
  public void setIntakeAngle(double angle) {
    m_lowerIntake.setPosition(angle);
  }

  // double vel: sets the velocity 
  // double acc: sets the acceleration 
  // spins the intake 
  public void spinIntakeMotor(double vel, double acc) {
    VelocityVoltage spinRequest1 = new VelocityVoltage(
    vel, acc, false, 0, 0,false, false, false);
    VelocityVoltage spinRequest2 = new VelocityVoltage(
    -vel, acc, false, 0, 0,false, false, false);
    m_spinIntake1.setControl(spinRequest1);
    m_spinIntake2.setControl(spinRequest2);
  }

  // Stops motors
  public void stopIntakeMotor() {
     m_spinIntake1.stopMotor();
  }

  // returns the position of the angle of the lowering motor
  public double getIntakePos() {
    return m_lowerIntake.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
