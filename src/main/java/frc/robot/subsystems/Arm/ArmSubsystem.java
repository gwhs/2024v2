// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_armMotor_ID, ArmConstants.ARM_armMotor_CAN);
    DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.ARM_armEncoder_CHANNEL);
    Constraints constraints = new Constraints(ArmConstants.ARM_VEL, ArmConstants.ARM_ACC);
    ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.ARM_kP, ArmConstants.ARM_kI, ArmConstants.ARM_kP, constraints);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
