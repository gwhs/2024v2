// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* starting position of arm is 0 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_moveIntakeArm; // motor of arm
  private TalonFX m_spinIntake; // motor of intake
  private Encoder m_Encoder;
  private DigitalInput m_sensor; 
  private VelocityVoltage spinRequest1;
  private double intakeMotorVelocity; 
  private double intakeMotorAcceleration; 
  
  // int lowerIntakeId: Id for lowerng motors for the intake
  // int spinIntakeId: Id for spining first intake motor 
  // initialized the encoder 
  // String can: String ID of canivore  
  public IntakeSubsystem(int lowerIntakeId, int spinIntakeId, int channel1, int channel2, int channel3, String can)  {
    m_moveIntakeArm = new TalonFX(lowerIntakeId, can); 
    m_spinIntake = new TalonFX(spinIntakeId, can);
    m_Encoder = new Encoder(channel1, channel2);
    m_sensor = new DigitalInput(channel3);
    this.intakeMotorVelocity = Constants.IntakeConstants.INTAKE_MOTOR_VELOCITY;
    this.intakeMotorAcceleration = Constants.IntakeConstants.INTAKE_MOTOR_ACCELERATION;
    
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = m_moveIntakeArm.getConfigurator().apply(configs);
      status2 = m_spinIntake.getConfigurator().apply(configs);
      if (status1.isOK() || status2.isOK()) break;
    }
    if(!status1.isOK() || !status2.isOK()) {
      System.out.println("Could not apply configs, error code: " + status1.toString());
      System.out.println("Could not apply configs, error code: " + status2.toString());
    }

  }

 
  // sets the angle of the intake motor
  public void setArmAngle(double angle) {
    if(angle < 0) { // minimum angle
      angle = 0;
    } 
    else if (angle > 120) { // maximum angle, need to update
      angle = 120;
    }

    // check falcon ticks
    double setAngle = (m_Encoder.getRaw() / 8132.0) + m_spinIntake.getPosition().getValue() / Constants.IntakeConstants.GEAR_RATIO;
    angle = angle - setAngle;

    m_moveIntakeArm.setPosition(angle);

  }

  // spin the intake motors
  public void spinIntakeMotor() {
    spinRequest1 = new VelocityVoltage(
      intakeMotorVelocity, intakeMotorAcceleration, false, 0, 0,false, false, false);
    m_spinIntake.setControl(spinRequest1);
  }
  
  // spin intake motors the opposite way
  public void rejectIntake() {
    spinRequest1 = new VelocityVoltage(
      intakeMotorVelocity * -1, intakeMotorAcceleration * -1, false, 0, 0,false, false, false);
      m_spinIntake.setControl(spinRequest1);
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
    return m_Encoder.getDistance();
    //return m_lowerIntake.getPosition().getValue();
  }

  // stop motor once note is in place, starts again once the arm position is brought up
  public boolean getSensor() {
    return m_sensor.get();
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
