// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* when intake arm position is down, that is at 0 degrees */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.UtilMotor;

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
  public boolean emergencyStop = false;

  /*
   * int lowerIntakeId: Id for lowerng motors for the intake
   * int spinIntakeId: Id for spining first intake motor
   * initialized the encoder
   * String can: String ID of canivore
   */
  public IntakeSubsystem(int lowerIntakeId, int spinIntakeId, String can) {

    m_moveIntakeArm = new TalonFX(lowerIntakeId, can);
    m_spinIntake = new TalonFX(spinIntakeId, can);
    m_Encoder = new DutyCycleEncoder(Constants.IntakeConstants.INTAKE_ENCODER_CHANNEL_ID);
    m_noteSensor = new DigitalInput(Constants.IntakeConstants.INTAKE_NOTESENSOR_CHANNEL_ID);

    UtilMotor.configMotor(m_moveIntakeArm, 0.11, 0.05, 0.01, 0.12, 12, 80, true);
    // UtilMotor.configMotor(m_spinIntake, 0, 0, 0, 0.12, 12, 80, true);
    UtilMotor.configMotorStatorCurrent(m_spinIntake, 60);
    UtilMotor.configMotorSupplyCurrent(m_spinIntake, 80);

    Shuffleboard.getTab("Intake").addDouble("Encoder Angle", () -> encoderGetAngle()).withWidget(BuiltInWidgets.kGraph)
        .withSize(3, 3)
        .withPosition(0, 0);

    Shuffleboard.getTab("Intake").addBoolean("Sensor value", () -> isNotePresent());
    m_Encoder.reset();
    // Logger.recordOutput("Intake/EncoderAngle", encoderGetAngle());
    // Logger.recordOutput("Intake/SensorValue", isNotePresent());
    // Logger.recordOutput("Intake/Motor/StatorCurrent", m_spinIntake.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake/Motor/SupplyCurrent", m_spinIntake.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake/Motor/TorqueCurrent", m_spinIntake.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake/Motor/Velocity", m_spinIntake.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Intake/ArmMotor/StatorCurrent", m_moveIntakeArm.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake/ArmMotor/SupplyCurrent", m_moveIntakeArm.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput("Intake/ArmMotor/TorqueCurrent", m_moveIntakeArm.getTorqueCurrent().getValueAsDouble());
  }

  // spin the intake motors, velocity is negative to intake note
  // velocity and accleration between -1.0 to 1.0
  public void spinIntakeMotor(double intakeMotorVelocity, double intakeMotorAcceleration) {
    // spinRequest1 = new VelocityVoltage(
    // -intakeMotorVelocity, intakeMotorAcceleration, true, 0, 0,false, false,
    // false);
    if (!emergencyStop) {
        m_spinIntake.set(-intakeMotorVelocity);
    }
  }

  // spin intake motors the opposite way, velocity is positive to reject intake
  public void rejectIntake(double intakeMotorVelocity, double intakeMotorAcceleration) {
    // spinRequest1 = new VelocityVoltage(
    // intakeMotorVelocity, intakeMotorAcceleration, true, 0, 0, false, false,
    // false);
    if (!emergencyStop) {
      m_spinIntake.set(intakeMotorVelocity);
    }
  }

  public void spinIntakeArm(double speed) {
    if (speed < -1) { // Will not be less than minimum angle
      speed = -1;
    } else if (speed > 1) { // Will not be greater than maximum angle
      speed = 1;
    }
    if (!isEmergencyStop()) {
      m_moveIntakeArm.set(speed);
    }
  }

  // stop intake motor
  public void stopIntakeMotors() {
    m_spinIntake.stopMotor();
  }

  // stop arm motor
  public void stopArmMotor() {
    m_moveIntakeArm.stopMotor();
  }

  public double encoderGetAngle() {
    return ((m_Encoder.getAbsolutePosition() * Constants.IntakeConstants.ROTATION_TO_DEGREES)
        - Constants.IntakeConstants.ENCODER_OFFSET);
  }

  // stop motor once note is in place, starts again once the arm position is
  // brought up
  public boolean isNotePresent() {
    return !m_noteSensor.get();
  }

  public boolean isEmergencyStop() {
    return !(m_Encoder.isConnected() && !emergencyStop);
  }
}
