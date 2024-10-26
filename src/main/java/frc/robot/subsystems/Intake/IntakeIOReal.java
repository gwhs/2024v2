// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOReal implements IntakeIO {
  /*
   * Hardware
   */
  private final TalonFX m_intakeArm = new TalonFX(IntakeConstants.INTAKE_ARM_ID, IntakeConstants.INTAKE_ARM_CAN);
  private final TalonFX m_intakeSpin = new TalonFX(IntakeConstants.INTAKE_SPIN_ID, IntakeConstants.INTAKE_SPIN_CAN);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.INTAKE_ENCODER_CHANNEL_ID);
  private final DigitalInput noteSensor = new DigitalInput(IntakeConstants.INTAKE_NOTE_SENSOR_CHANNEL_ID);

  /*
   * Status Signals from CTRE motors
   */
  private final StatusSignal<AngularVelocity> spinVelocity = m_intakeSpin.getVelocity();
  private final StatusSignal<Temperature> spinTemp = m_intakeSpin.getDeviceTemp();
  private final StatusSignal<Current> spinSupplyCurrent = m_intakeSpin.getSupplyCurrent();
  private final StatusSignal<Current> spinStatorCurrent = m_intakeSpin.getStatorCurrent();
  private final StatusSignal<Voltage> spinAppliedVoltage = m_intakeSpin.getMotorVoltage();

  private final StatusSignal<Angle> armPosition = m_intakeArm.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = m_intakeArm.getVelocity();
  private final StatusSignal<Temperature> armTemp = m_intakeArm.getDeviceTemp();
  private final StatusSignal<Current> armSupplyCurrent = m_intakeArm.getSupplyCurrent();
  private final StatusSignal<Current> armStatorCurrent = m_intakeArm.getStatorCurrent();
  private final StatusSignal<Voltage> armAppliedVoltage = m_intakeArm.getMotorVoltage();

  /*
   * Set up for logging values to network table
   */
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Intake");

  private final BooleanPublisher armConnected = nt.getBooleanTopic("Arm Connected").publish();
  private final BooleanPublisher spinConnected = nt.getBooleanTopic("Spin Connected").publish();

  private final DoublePublisher nt_spinTemp = nt.getDoubleTopic("Spin Motor/Temp").publish();
  private final DoublePublisher nt_spinSupplyCurrent = nt.getDoubleTopic("Spin Motor/Supply Current").publish();
  private final DoublePublisher nt_spinStatorCurrent = nt.getDoubleTopic("Spin Motor/Stator Current").publish();
  private final DoublePublisher nt_spinAppliedVoltage = nt.getDoubleTopic("Spin Motor/Voltage").publish();

  private final DoublePublisher nt_armPosition = nt.getDoubleTopic("Arm Motor/Position").publish();
  private final DoublePublisher nt_armVelocity = nt.getDoubleTopic("Arm Motor/Velocity").publish();
  private final DoublePublisher nt_armTemp = nt.getDoubleTopic("Arm Motor/Temp").publish();
  private final DoublePublisher nt_armSupplyCurrent = nt.getDoubleTopic("Arm Motor/Supply Current").publish();
  private final DoublePublisher nt_armStatorCurrent = nt.getDoubleTopic("Arm Motor/Stator Current").publish();
  private final DoublePublisher nt_armAppliedVoltage = nt.getDoubleTopic("Arm Motor/Voltage").publish();

  public IntakeIOReal() {
    /*
     * Motor configs for intake arm motor
     */
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_intakeArm.getConfigurator().apply(config);

    /*
     * Motor configs for intake spin motor
     */
    config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_intakeSpin.getConfigurator().apply(config);

  }

  public double getIntakeArmAngle() {
    return Units.rotationsToDegrees(encoder.get()) - IntakeConstants.ENCODER_OFFSET;
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
    /*
     * Refresh all status signals
     */
    spinConnected.set(BaseStatusSignal.refreshAll(
        spinVelocity,
        spinTemp,
        spinSupplyCurrent,
        spinStatorCurrent,
        spinAppliedVoltage).isOK());

    armConnected.set(BaseStatusSignal.refreshAll(
        armPosition,
        armVelocity,
        armTemp,
        armSupplyCurrent,
        armStatorCurrent,
        armAppliedVoltage).isOK());

    /*
     * Log status signal values to network table
     */
    nt_spinTemp.set(spinTemp.getValue().in(Fahrenheit));
    nt_spinSupplyCurrent.set(spinSupplyCurrent.getValue().in(Amp));
    nt_spinStatorCurrent.set(spinStatorCurrent.getValue().in(Amp));
    nt_spinAppliedVoltage.set(spinAppliedVoltage.getValue().in(Volt));

    nt_armPosition.set(armPosition.getValue().in(Degrees));
    nt_armVelocity.set(armVelocity.getValue().in(DegreesPerSecond));
    nt_armTemp.set(armTemp.getValue().in(Fahrenheit));
    nt_armSupplyCurrent.set(armSupplyCurrent.getValue().in(Amp));
    nt_armStatorCurrent.set(armStatorCurrent.getValue().in(Amp));
    nt_armAppliedVoltage.set(armAppliedVoltage.getValue().in(Volt));
  }
}
