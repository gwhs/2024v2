// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* when intake arm position is down, that is at 0 degrees */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
  private double intakeMotorVelocity; 
  private double intakeMotorAcceleration; 
  
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
    
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.05; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.01; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 3.5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.7; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.2; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = m_moveIntakeArm.getConfigurator().apply(configs);
      status2 = m_spinIntake.getConfigurator().apply(configs);
      if (status1.isOK() && status2.isOK()) break;
    }
    if(!status1.isOK() || !status2.isOK()) {
      System.out.println("Could not apply configs, error code: " + status1.toString());
      System.out.println("Could not apply configs, error code: " + status2.toString());
    }

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
    m_moveIntakeArm.set(speed);
  }  

  public void setIntakeArmAngle(double angle) {
    System.out.println("set intake angle method works");
    
    if(angle < 0) {
      angle = 0;
    }
    else if (angle > Constants.IntakeConstants.MAX_ARM_ANGLE) {
      angle = Constants.IntakeConstants.MAX_ARM_ANGLE;
    }

    setGoal(angle);
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
    return ((m_Encoder.get() * Constants.IntakeConstants.ROTATION_TO_DEGREES) - Constants.IntakeConstants.ENCODER_OFFSET); 
  }

  // stop motor once note is in place, starts again once the arm position is brought up
  public boolean isNotePresent() {
    return m_noteSensor.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    spinIntakeArm(output);
    System.out.println(output);
  }

  @Override
  public double getMeasurement() {
    System.out.println("encoder: " + encoderGetAngle());
    return encoderGetAngle();
  }
}

