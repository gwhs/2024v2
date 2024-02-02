// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* starting position of arm is 0 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  // int spinIntake1Id: Id for spining first intake motor 
  // int spinIntake2Id: Id for spining second intake motor 
  // initialized the encoder 
  // String can: String ID of canivore  
  public IntakeSubsystem(int lowerIntakeId, int spinIntake1Id, int spinIntake2Id, int channel1, int channel2, int channel3, String can)  {
    m_moveIntakeArm = new TalonFX(lowerIntakeId, can); 
    m_spinIntake = new TalonFX(spinIntake1Id, can);
    m_Encoder = new Encoder(channel1, channel2);
    m_sensor = new DigitalInput(channel3);
    this.intakeMotorVelocity = Constants.IntakeConstants.INTAKE_MOTOR_VELOCITY;
    this.intakeMotorAcceleration = Constants.IntakeConstants.INTAKE_MOTOR_ACCELERATION;
  }
 
  //Sets the angle for the intake motor
  public void setArmAngle(double angle) {
    if(angle < 0) { // minimum angle
      angle = 0;
    } 
    else if (angle > 120) { // maximum angle, need to update
      angle = 120;
    }

    // check falcon ticks
    double setAngle = m_Encoder.getRaw() / 360 * Constants.IntakeConstants.FALCON_TICKS * Constants.IntakeConstants.GEAR_RATIO + m_spinIntake.getPosition().getValue();
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

  // Stops intake motors
  public void stopIntakeMotors() {
    m_spinIntake.stopMotor();
  }

  public void stopArmMotor() {
    m_moveIntakeArm.stopMotor();
 }

  // returns the position of the angle of the lowering motor
  public double getArmPos() {
    return m_Encoder.getDistance();
    //return m_lowerIntake.getPosition().getValue();
  }

  // stops motor once note is in place, starts again once the arm position is brought up
  public boolean getSensor(){
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
