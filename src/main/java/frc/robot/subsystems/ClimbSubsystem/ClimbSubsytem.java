// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class ClimbSubsytem extends SubsystemBase {
    private TalonFX m_leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private TalonFX m_rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
    private Constraints constraints = new Constraints(180.0, 300.0);


    private ProfiledPIDController pidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP, ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);






}