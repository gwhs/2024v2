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

public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_arm;
  private final Encoder m_encoder;
  //Pizza Box Motor
  private TalonFX m_pizzaBox;
  private double pizzaBoxVel;
  private double pizzaBoxAcc;
  private VelocityVoltage spinPizzaBoxMotorRequest;  

  public ArmSubsystem(int armId, String armCanbus, int pizzaBoxId, String pizzaBoxCanbus, int channel1, int channel2)
  {
      m_arm = new TalonFX(armId, armCanbus);
      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_encoder = new Encoder(channel1, channel2, false, Counter.EncodingType.k4X);


      TalonFXConfiguration configs = new TalonFXConfiguration();
      /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
      configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
      configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
      configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
      configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
      // Peak output of 8 volts
      configs.Voltage.PeakForwardVoltage = 8;
      configs.Voltage.PeakReverseVoltage = -8;
      
      /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
      configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
      configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
      configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  
      // Peak output of 40 amps
      configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
  
      /* Retry config apply up to 5 times, report if failure */
      StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
      StatusCode motorStatusArm = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        motorStatus = m_pizzaBox.getConfigurator().apply(configs);
        //motorStatusArm = m_arm.getConfigurator().apply(configs);
        if (motorStatus .isOK() && motorStatusArm .isOK()) break;
      }
      if(!motorStatus.isOK()) {
        System.out.println("Could not apply configs, error code: " + motorStatus.toString());
      }
      // if(!motorStatusArm.isOK()) {
      //   System.out.println("Could not apply configs, error code: " + motorStatusArm.toString());
      // }


      
      
      // m_arm.getPosition().setUpdateFrequency(5);
      // //Try to get it related to the TalonFXConfiguration
      // var slot0Configs = new Slot0Configs(); 
      // //Draft
      // slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
      // slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
      // //Setted to zero for now just to see it compiles and builds
      // slot0Configs.kP = 0;
      // slot0Configs.kI = 0;
      // slot0Configs.kD = 0;

      // m_arm.getConfigurator().apply(talonFXConfigs, 0.03);

  
      m_encoder.reset();
      m_encoder.setSamplesToAverage(5);
      m_encoder.setDistancePerPulse(1. / 256.);
      m_encoder.setMinRate(1.0);
    
  }

  // Sets arm angle in degrees with given velocity and acceleration
  public void setAngle(double angle, double vel, double accel) {
  //Pos will be based on motor 
  //Make sure angle is between 0 and 270 degrees!

  if(angle < 0) { //Will not be less than minimum angle
    angle = 0;
  }
  else if (angle > 270) { // Will not be greater than maximum angle
    angle = 270;
  }

  double adjustedAngle = (((angle - encoderGetAngle() + m_arm.getPosition().getValue())) * Constants.Arm.GEAR_RATIO)/360;
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
  //m_arm.set(10);
}

  //Spins "Pizzabox" motor: velocity in rotations/sec and acceleration in rotations/sec^2
  public void spinPizzaBoxMotor(double velocity, double acceleration){
    pizzaBoxVel = velocity;
    pizzaBoxAcc = acceleration;
    spinPizzaBoxMotorRequest = new VelocityVoltage(pizzaBoxVel, pizzaBoxAcc, true, 0, 0, false, false, false);
    m_pizzaBox.setControl(spinPizzaBoxMotorRequest);
    //m_pizzaBox.set(pizzaBoxVel);
  }

  //Resets arm angle back to 0
  public void resetPosition() {
    setAngle(0.0, 0.0, 0.0); 
  }

  //Stops arm motor
  public void stopArmMotor() {
    m_arm.stopMotor();
 }

 public void stopPizzaBoxMotor() {
  m_pizzaBox.stopMotor();
}

 public double getArmAngle(){
  return m_arm.getPosition().getValue()/Constants.Arm.GEAR_RATIO * 360;
 }

 /* The pizza box is the motor on the holding container on the arm*/ 
 public double getPizzaBoxAngle(){
  return m_pizzaBox.getPosition().getValue() * 360;
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getRaw()/8132. * -360;
  }

  public void encoderReset() {
    m_encoder.reset();
  }
//Check
  public boolean encoderGetStopped() {
    return m_encoder.getStopped();
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

/*Previously setAngle that turned to spinArmMotor */
// public void setAngle(double angle, double vel, double accel) {
//   //Pos will be based on motor 
//   //Make sure angle is between 0 and 270 degrees!

//   if(angle < 0) { //Will not be less than minimum angle
//     angle = 0;
//   }
//   else if (angle > 270) { // Will not be greater than maximum angle
//     angle = 270;
//   }

//   double adjustedAngle = ((angle - encoderGetAngle() + m_arm.getPosition().getValue())) * Constants.Arm.GEAR_RATIO;
//   MotionMagicVoltage m_smoothArmMovement = new MotionMagicVoltage(adjustedAngle/360, true, 0, 0, false, true, false);

//   var motionMagicConfigs = talonFXConfigs.MotionMagic;
//   motionMagicConfigs.MotionMagicCruiseVelocity = vel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10;
//   motionMagicConfigs.MotionMagicAcceleration = accel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10; 

//    m_arm.getConfigurator().apply(talonFXConfigs, 0.03); 

//   m_arm.setControl(m_smoothArmMovement);
//   //m_arm.set(10);
// }