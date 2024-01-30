// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 
import com.ctre.phoenix6.signals.ControlModeValue; 
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_arm;
  private final Encoder m_encoder;
  //Pizza Box Motor
  private TalonFX m_pizzaBox;
  private double pizzaBoxVel;
  private double pizzaBoxAcc;
  private VelocityVoltage spinPizzaBoxMotorRequest;  
  TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  public ArmSubsystem(int armId, String armCanbus, int pizzaBoxId, String pizzaBoxCanbus, int channel1, int channel2)
  {
      m_arm = new TalonFX(armId, armCanbus);
      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_encoder = new Encoder(channel1, channel2, false, Counter.EncodingType.k4X);



      m_arm.getPosition().setUpdateFrequency(5);
      //Try to get it related to the TalonFXConfiguration
      var slot0Configs = new Slot0Configs(); 
      //Draft
      slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
      slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
      //Setted to zero for now just to see it compiles and builds
      slot0Configs.kP = 0;
      slot0Configs.kI = 0;
      slot0Configs.kD = 0;

      m_arm.getConfigurator().apply(talonFXConfigs, 0.03);

  
      m_encoder.reset();
      m_encoder.setSamplesToAverage(5);
      m_encoder.setDistancePerPulse(1. / 256.);
      m_encoder.setMinRate(1.0);
    
  }

  // Sets arm angle with given velocity and acceleration
  public void setAngle(double angle, double vel, double accel) {
    //Pos will be based on motor 
    //Make sure angle is between 0 and 270 degrees!

    if(angle < 0) { //Will not be less than minimum angle
      angle = 0;
    }
    else if (angle > 270) { // Will not be greater than maximum angle
      angle = 270;
    }

    double adjustedAngle = ((angle - encoderGetAngle() + m_arm.getPosition().getValue())) * Constants.Arm.GEAR_RATIO;
    MotionMagicVoltage m_smoothArmMovement = new MotionMagicVoltage(adjustedAngle/360, true, 0, 0, false, true, false);

    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = vel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10;
    // motionMagicConfigs.MotionMagicAcceleration = accel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10; 

    // m_arm.getConfigurator().apply(talonFXConfigs, 0.03); 

    m_arm.setControl(m_smoothArmMovement);
  }

  //Spins "Pizzabox" motor
  public void spinPizzaBoxMotor(double velocity, double acceleration){
    pizzaBoxVel = velocity;
    pizzaBoxAcc = acceleration;
    spinPizzaBoxMotorRequest = new VelocityVoltage(velocity, acceleration, true, 0, 0, false, false, false);
    m_pizzaBox.setControl(spinPizzaBoxMotorRequest);
  }

  //Resets arm angle back to 0
  public void resetPosition() {
    setAngle(0.0, 0.0, 0.0); 
  }

  //Stops arm motor
  public void stopArmMotor() {
    m_arm.stopMotor();
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getRaw()/8132 * 360;
  }

  public void encoderReset() {
    m_encoder.reset();
  }

  public boolean encoderGetStopped() {
    return m_encoder.getStopped();
  }

 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ticks = m_encoder.get();

    double rawAngle = (-m_encoder.getRaw() / 8192. * 360.);
    Logger.getInstance().recordOutput("/Angle", rawAngle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
