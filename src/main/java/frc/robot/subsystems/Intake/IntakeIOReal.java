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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOReal implements IntakeIO {
  private TalonFX m_intakeArm = new TalonFX(IntakeConstants.INTAKE_ARM_ID, IntakeConstants.INTAKE_ARM_CAN);
  private TalonFX m_intakeSpin = new TalonFX(IntakeConstants.INTAKE_SPIN_ID, IntakeConstants.INTAKE_SPIN_CAN);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.INTAKE_ENCODER_CHANNEL_ID);
  private DigitalInput noteSensor = new DigitalInput(IntakeConstants.INTAKE_NOTE_SENSOR_CHANNEL_ID);

  StatusSignal<Double> spinVelocity = m_intakeSpin.getVelocity();
  StatusSignal<Double> spinTemp = m_intakeSpin.getDeviceTemp();
  StatusSignal<Double> spinSupplyCurrent = m_intakeSpin.getSupplyCurrent();
  StatusSignal<Double> spinStatorCurrent = m_intakeSpin.getStatorCurrent();
  StatusSignal<Double> spinAppliedVoltage = m_intakeSpin.getMotorVoltage();

  StatusSignal<Double> armPosition = m_intakeArm.getPosition();
  StatusSignal<Double> armVelocity = m_intakeArm.getVelocity();
  StatusSignal<Double> armTemp = m_intakeArm.getDeviceTemp();
  StatusSignal<Double> armSupplyCurrent = m_intakeArm.getSupplyCurrent();
  StatusSignal<Double> armStatorCurrent = m_intakeArm.getStatorCurrent();
  StatusSignal<Double> armAppliedVoltage = m_intakeArm.getMotorVoltage();

  DoublePublisher nt_intakeSpin_temp = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Spin Motor/Temp")
      .publish();
  DoublePublisher nt_intakeSpin_supplyCurrent = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Spin Motor/Supply Current").publish();
  DoublePublisher nt_intakeSpin_statorCurrent = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Spin Motor/Stator Current").publish();
  DoublePublisher nt_intakeSpin_appliedVoltage = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Spin Motor/Applied Voltage").publish();

  DoublePublisher nt_intakeArm_position = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Arm Motor/Position")
      .publish();
  DoublePublisher nt_intakeArm_velocity = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Arm Motor/Velocity")
      .publish();
  DoublePublisher nt_intakeArm_temp = NetworkTableInstance.getDefault().getDoubleTopic("Intake/Arm Motor/Temp")
      .publish();
  DoublePublisher nt_intakeArm_supplyCurrent = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Arm Motor/Supply Current").publish();
  DoublePublisher nt_intakeArm_statorCurrent = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Arm Motor/Stator Current").publish();
  DoublePublisher nt_intakeArm_appliedVoltage = NetworkTableInstance.getDefault()
      .getDoubleTopic("Intake/Arm Motor/Applied Voltage").publish();

  public IntakeIOReal() {
    /*
     * Motor configs for intake arm motor
     */
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator intakeArmConfigurator = m_intakeArm.getConfigurator();
    intakeArmConfigurator.apply(motorOutput);
    intakeArmConfigurator.apply(currentConfig);

    /*
     * Motor configs for intake spin motor
     */
    motorOutput = new MotorOutputConfigs();
    currentConfig = new CurrentLimitsConfigs();

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(100);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator intakeSpinConfigurator = m_intakeSpin.getConfigurator();
    intakeSpinConfigurator.apply(motorOutput);
    intakeSpinConfigurator.apply(currentConfig);

  }

  public double getIntakeArmAngle() {
    return Units.rotationsToDegrees(encoder.getAbsolutePosition()) - IntakeConstants.ENCODER_OFFSET;
  }

  public void setArmSpeed(double speed) {
    m_intakeArm.set(speed);
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
        spinVelocity,
        spinTemp,
        spinSupplyCurrent,
        spinStatorCurrent,
        spinAppliedVoltage,

        armPosition,
        armVelocity,
        armTemp,
        armSupplyCurrent,
        armStatorCurrent,
        armAppliedVoltage);

    nt_intakeSpin_temp.set(spinTemp.getValueAsDouble());
    nt_intakeSpin_supplyCurrent.set(spinSupplyCurrent.getValueAsDouble());
    nt_intakeSpin_statorCurrent.set(spinStatorCurrent.getValueAsDouble());
    nt_intakeSpin_appliedVoltage.set(spinAppliedVoltage.getValueAsDouble());

    nt_intakeArm_position.set(armPosition.getValueAsDouble());
    nt_intakeArm_velocity.set(armVelocity.getValueAsDouble());
    nt_intakeArm_temp.set(armTemp.getValueAsDouble());
    nt_intakeArm_supplyCurrent.set(armSupplyCurrent.getValueAsDouble());
    nt_intakeArm_statorCurrent.set(armStatorCurrent.getValueAsDouble());
    nt_intakeArm_appliedVoltage.set(armAppliedVoltage.getValueAsDouble());
  }
}
