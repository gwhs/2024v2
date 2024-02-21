// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 
import com.ctre.phoenix6.signals.ControlModeValue; 
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Servo;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_arm;
  private Encoder m_encoder;
  private TalonFX m_pizzaBox;
  private Servo m_servo;
  
    

  public ArmSubsystem(int armId, String armCanbus, int pizzaBoxId, String pizzaBoxCanbus, int channel1, int channel2, int channelServo)
  {
      m_arm = new TalonFX(armId, armCanbus);
      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_encoder = new Encoder(channel1, channel2, false, Counter.EncodingType.k4X);
      m_servo = new Servo(channelServo);


      TalonFXConfiguration configs = new TalonFXConfiguration();
      /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
      configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
      configs.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
      configs.Slot0.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
      configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
      // Peak output of 8 volts
      configs.Voltage.PeakForwardVoltage = 15;
      configs.Voltage.PeakReverseVoltage = -15;
      
      /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
      configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
      configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
      configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  
      // Peak output of 40 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 50;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -50;
  
      /* Retry config apply up to 5 times, report if failure */
      StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
      StatusCode motorStatusArm = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        motorStatusArm = m_arm.getConfigurator().apply(configs);
        if (motorStatusArm .isOK()) break;
      }      
      for (int i = 0; i < 5; ++i) {
        motorStatus = m_pizzaBox.getConfigurator().apply(configs);
        if (motorStatus .isOK()) break;
      }
      if(!motorStatus.isOK()) {
        System.out.println("Could not apply configs, error code: " + motorStatus.toString());
      }
      if(!motorStatusArm.isOK()) {
        System.out.println("Could not apply configs, error code: " + motorStatusArm.toString());
      }
  
      m_encoder.reset();
      m_encoder.setSamplesToAverage(5);
      m_encoder.setDistancePerPulse(1. / 256.);
      m_encoder.setMinRate(1.0);
    
  }

  // Sets arm angle in degrees with given velocity and acceleration
  public void setAngle(double angle, double vel, double accel) {

    if(angle < Constants.Arm.ARM_MIN_ANGLE) { //Will not be less than minimum angle
      angle = Constants.Arm.ARM_MIN_ANGLE;
    }
    else if (angle > Constants.Arm.ARM_MAX_ANGLE) { // Will not be greater than maximum angle
      angle = Constants.Arm.ARM_MAX_ANGLE;
    }

    double adjustedAngle = (((angle - encoderGetAngle() + getArmAngle())) * Constants.Arm.GEAR_RATIO)/Constants.Arm.ROTATION_TO_DEGREES;
    MotionMagicVoltage m_smoothArmMovement = new MotionMagicVoltage(adjustedAngle, false, 0, 0, false, false, false);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.Slot0.kS = .24;
    talonFXConfigs.Slot0.kV = .12;
    talonFXConfigs.Slot0.kP = 4.8;
    talonFXConfigs.Slot0.kI = 0;
    talonFXConfigs.Slot0.kD = .1;


    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = vel;
    motionMagicConfigs.MotionMagicAcceleration = accel; 
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    m_arm.getConfigurator().apply(talonFXConfigs, 0.03);

    m_smoothArmMovement.Slot = 0;

    m_arm.setControl(m_smoothArmMovement);
  }

  //Spins "Pizzabox" motor: velocity in rotations/sec and acceleration in rotations/sec^2
  public void spinPizzaBoxMotor(double velocity, double acceleration){
    VelocityVoltage spinPizzaBoxMotorRequest = new VelocityVoltage(velocity, acceleration, true, 0, 0, false, false, false);
    m_pizzaBox.setControl(spinPizzaBoxMotorRequest);
  }
  //Sets the position of the Servo motor on the pizza box
  public void setServoAngle(double angle) {
    m_servo.setAngle(angle);
  }

  //Returns the servo postion from 0.0 to 1.0 (0 degrees to 180 degrees)
  public double getServoAngle() {
    return m_servo.getAngle();
  }

  //Stops arm motor
  public void stopArmMotor() {
    m_arm.stopMotor();
 }

 //Stops pizzaBox motor
 public void stopPizzaBoxMotor() {
  m_pizzaBox.stopMotor();
}

 public double getArmAngle(){
  return m_arm.getPosition().getValue()/Constants.Arm.GEAR_RATIO * Constants.Arm.ROTATION_TO_DEGREES;
 }

 /* The pizza box is the motor on the holding container on the arm*/ 
 public double getPizzaBoxAngle(){
  return m_pizzaBox.getPosition().getValue() * Constants.Arm.ROTATION_TO_DEGREES;
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getRaw()/Constants.Arm.ENCODER_RAW_TO_ROTATION * -Constants.Arm.ROTATION_TO_DEGREES;
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