// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOReal implements IntakeIO {
  private TalonFX m_intakeArm = new TalonFX(IntakeContants.INTAKE_ARM_ID, IntakeContants.INTAKE_ARM_CAN);
  private TalonFX m_intakeSpin = new TalonFX(IntakeContants.INTAKE_SPIN_ID, IntakeContants.INTAKE_SPIN_CAN);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeContants.INTAKE_ENCODER_CHANNEL_ID);
  private DigitalInput noteSensor = new DigitalInput(IntakeContants.INTAKE_NOTE_SENSOR_CHANNEL_ID);

  StatusSignal<Double> spinVelocity = m_intakeSpin.getVelocity();

  public double getIntakeArmAngle() {
    return Units.rotationsToDegrees(encoder.getAbsolutePosition()) - IntakeContants.ENCODER_OFFSET;
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
      spinVelocity
    );
  }
}