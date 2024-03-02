// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* when intake arm position is down, that is at 0 degrees */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.UtilMotor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_moveIntakeArm; // motor of arm
  private TalonFX m_spinIntake; // motor of intake
  private DutyCycleEncoder m_Encoder;
  private DigitalInput m_noteSensor; // sensor to check if note is present in intake
  private VelocityVoltage spinRequest1; 
  public boolean emergencyStop = false;

  
  /*  
    * int lowerIntakeId: Id for lowerng motors for the intake
    * int spinIntakeId: Id for spining first intake motor 
    * initialized the encoder 
    * String can: String ID of canivore  
  */
  public IntakeSubsystem(int lowerIntakeId, int spinIntakeId, String can)  {
    
    m_moveIntakeArm = new TalonFX(lowerIntakeId, can); 
    m_spinIntake = new TalonFX(spinIntakeId, can);
    m_Encoder = new DutyCycleEncoder(Constants.IntakeConstants.INTAKE_ENCODER_CHANNEL_ID);
    m_noteSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_NOTESENSOR_CHANNEL_ID); 

    UtilMotor.configMotor(m_moveIntakeArm, 0.11, 0.05, 0.01,  0.12, 80, 40, true);
    
    Shuffleboard.getTab("Intake").addDouble("Encoder Angle", ()->encoderGetAngle()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3)
    .withPosition(0, 0);;
    Shuffleboard.getTab("Intake").addDouble("Motor Angle", ()->getArmPos()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3)
    .withPosition(3, 0);;
  }

  // spin the intake motors, velocity is negative to intake note
  public void spinIntakeMotor(int intakeMotorVelocity, int intakeMotorAcceleration) {
    spinRequest1 = new VelocityVoltage(
      -intakeMotorVelocity, intakeMotorAcceleration, true, 0, 0,false, false, false);
    m_spinIntake.setControl(spinRequest1);
  }
  
  // spin intake motors the opposite way, velocity is positive to reject intake
  public void rejectIntake(int intakeMotorVelocity, int intakeMotorAcceleration) {
    spinRequest1 = new VelocityVoltage(
      intakeMotorVelocity, intakeMotorAcceleration, true, 0, 0, false, false, false);
      m_spinIntake.setControl(spinRequest1);
  }

  public void spinIntakeArm(double speed) {
  if(speed < -1) { // Will not be less than minimum angle
    speed = -1;
  }
  else if (speed > 1) { // Will not be greater than maximum angle
    speed = 1;
  }
  if(m_Encoder.isConnected() && !emergencyStop) {
      m_moveIntakeArm.set(speed);
    }
    
  }  

  public void setIntakeArmAngle(double angle) {
    System.out.println("set intake angle method works");
    
    if(angle < 0) {
      angle = 0;
    }
    else if (angle > Constants.IntakeConstants.MAX_ARM_ANGLE) {
      angle = Constants.IntakeConstants.MAX_ARM_ANGLE;
    }

    //setGoal(angle);
  }

  // stop intake motor
  public void stopIntakeMotors() {
    m_spinIntake.stopMotor();
  }

  // stop arm motor 
  public void stopArmMotor() {
    m_moveIntakeArm.stopMotor();
  }

  // returns the position of the angle of the lowering motor
  public double getArmPos() {
    return m_moveIntakeArm.getPosition().getValue()/Constants.IntakeConstants.GEAR_RATIO * Constants.IntakeConstants.ROTATION_TO_DEGREES;
  }

  public double encoderGetAngle() {
    return ((m_Encoder.getAbsolutePosition() * Constants.IntakeConstants.ROTATION_TO_DEGREES) - Constants.IntakeConstants.ENCODER_OFFSET); 
  }

  // stop motor once note is in place, starts again once the arm position is brought up
  public boolean isNotePresent() {
    return m_noteSensor.get();
  }

  public boolean isEmergencyStop()
  {
    return m_Encoder.isConnected() && !emergencyStop;
  }
  
  @Override
  public void periodic() {
    if(!(m_Encoder.isConnected()) || emergencyStop) {
      m_moveIntakeArm.stopMotor(); 
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // @Override
  // public void useOutput(double output, TrapezoidProfile.State setpoint) {
  //   spinIntakeArm(output);
  //   System.out.println(output);
  // }

  // @Override
  // public double getMeasurement() {
  //   System.out.println("encoder: " + encoderGetAngle());
  //   return encoderGetAngle();
  // }
}

