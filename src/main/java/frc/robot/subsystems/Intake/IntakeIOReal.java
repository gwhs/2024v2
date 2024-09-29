// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOReal implements IntakeIO {
  private TalonFX m_intakeArm = new TalonFX(IntakeConstants.INTAKE_ARM_ID, IntakeConstants.INTAKE_ARM_CAN);
  private TalonFX m_intakeSpin = new TalonFX(IntakeConstants.INTAKE_SPIN_ID, IntakeConstants.INTAKE_SPIN_CAN);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.INTAKE_ENCODER_CHANNEL_ID);
  private DigitalInput noteSensor = new DigitalInput(IntakeConstants.INTAKE_NOTE_SENSOR_CHANNEL_ID);

  StatusSignal<Double> spinVelocity = m_intakeSpin.getVelocity();

  public IntakeIOReal() {
    /*
     * Motor configs for intake arm motor
     */
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfigurator intakeArmConfigurator = m_intakeArm.getConfigurator();
    intakeArmConfigurator.apply(motorOutput);
    intakeArmConfigurator.apply(currentConfig);

    /*
     * Motor configs for intake spin motor
     */
    motorOutput = new MotorOutputConfigs();
    currentConfig = new CurrentLimitsConfigs();

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator intakeSpinConfigurator = m_intakeArm.getConfigurator();
    intakeSpinConfigurator.apply(motorOutput);
    intakeSpinConfigurator.apply(currentConfig);

  }

  public double getIntakeArmAngle() {
    return Units.rotationsToDegrees(encoder.getAbsolutePosition()) - IntakeConstants.ENCODER_OFFSET;
  }

  public void setArmSpeed(double speed) {
    m_intakeArm.set(-speed);
  }

  public void setSpinSpeed(double speed) {
    m_intakeSpin.set(speed);
  }

  public boolean getNoteSensor() {
    return !noteSensor.get();
  }

  public double getSpinSpeed() {
    return spinVelocity.getValueAsDouble();
  }

  public void update() {
    BaseStatusSignal.refreshAll(
        spinVelocity);
  }
}
