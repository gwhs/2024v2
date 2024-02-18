// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 * 
 * 
 * USES setGoal to spin
 * 
 * 
 * 
 */


//
package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController; 

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 
import com.ctre.phoenix6.signals.ControlModeValue; 
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Counter;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class ArmSubsystem extends ProfiledPIDSubsystem {
  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private TalonFX m_pizzaBox;
  private Servo m_servo;
  
  

  public ArmSubsystem(int armId, String armCanbus, int pizzaBoxId, String pizzaBoxCanbus, int channel1, int channelServo)
  {
    super(new ProfiledPIDController(.005, .0, 0, new Constraints(.05, .5)));
    getController().setTolerance(5);

    getController().enableContinuousInput(0, 360);
    //TrapezoidProfile either velocity or position
      m_arm = new TalonFX(armId, armCanbus);
      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_encoder = new DutyCycleEncoder(channel1);
      m_servo = new Servo(channelServo);
      
      

      // Needed? PIDController ArmPID = new PIDController(0, 0, 0);


      TalonFXConfiguration configs = new TalonFXConfiguration();
      /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
      //initial ramp up
      configs.Slot0.kP = .5; // An error of 1 rotation per second results in 2V output
      //how much it nudges towards target
      configs.Slot0.kI = 0.7; // An error of 1 rotation per second increases output by 0.5V every second
      //Close to value, how aggressive to slow down
      configs.Slot0.kD = 0.2; // A change of 1 rotation per second squared results in 0.01 volts output
      
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

    Shuffleboard.getTab("Arm").addDouble("Encoder Angle", ()->encoderGetAngle()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3)
    .withPosition(0, 0);
    Shuffleboard.getTab("Arm").addDouble("Motor Angle", ()->getArmAngle()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3)
    .withPosition(3, 0);

    Shuffleboard.getTab("Arm").add("Arm PID", this.getController());
  }

  //Looking at the left of the robot, counterclockwise arm spin is positive
  // Sets arm angle in degrees with given velocity and acceleration
  //VelocityVoltage​(double Velocity, double Acceleration, boolean EnableFOC, double FeedForward, int Slot, boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
 public void spinArm(double speed)
 {
  if(speed < -1) { //Will not be less than minimum angle
    speed = -1;
  }
  else if (speed > 1) { // Will not be greater than maximum angle
    speed = 1;
  }
  m_arm.set(speed);
 }

 public void targetArmAngle(double angle)
 {
  System.out.println("TargetArmAngle works, angle value : " + angle);
  setGoal(angle);
 }


  public void setAngle(double angle, double vel, double accel) {

    if(angle < Constants.Arm.ARM_MIN_ANGLE) { //Will not be less than minimum angle
      angle = Constants.Arm.ARM_MIN_ANGLE;
    }
    else if (angle > Constants.Arm.ARM_MAX_ANGLE) { // Will not be greater than maximum angle
      angle = Constants.Arm.ARM_MAX_ANGLE;
    }

    double wantedAngle = angle;
    double encoderAngle = encoderGetAngle();
    double armAngle = getArmAngle();
    double deltaAngle = wantedAngle - encoderAngle;
    double neededArmRotation = deltaAngle / 360; 
    double neededMotorRotation = neededArmRotation * Constants.Arm.GEAR_RATIO;

//ORIGINAl ADJUSTED ANGLE    double adjustedAngle = (((angle - encoderGetAngle() + getArmAngle())) * Constants.Arm.GEAR_RATIO)/Constants.Arm.ROTATION_TO_DEGREES;
//Testing
   double adjustedAngle = (((angle - encoderGetAngle() + getArmAngle())) * Constants.Arm.GEAR_RATIO)/Constants.Arm.ROTATION_TO_DEGREES;
//Testing

    System.out.println("NeededMotorRotation will be:  " + neededMotorRotation + " DeltaAngle will be : " + deltaAngle);
    //MotionMagicVoltage m_smoothArmMovement = new MotionMagicVoltage(neededMotorRotation, false, 0, 0, false, false, false);
    //Testing
    PositionVoltage m_smoothArmMovement = new PositionVoltage(adjustedAngle, vel , false, 0, 0, false, false, false);
    //Testing

    // var talonFXConfigs = new TalonFXConfiguration();
    // talonFXConfigs.Slot0.kS = .24;
    // talonFXConfigs.Slot0.kV = .12;
    // talonFXConfigs.Slot0.kP = 4.8;
    // talonFXConfigs.Slot0.kI = 0;
    // talonFXConfigs.Slot0.kD = .1;


    // // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // // motionMagicConfigs.MotionMagicCruiseVelocity = vel;
    // // motionMagicConfigs.MotionMagicAcceleration = accel; 
    // // motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    // m_arm.getConfigurator().apply(talonFXConfigs, 0.03);

    // m_smoothArmMovement.Slot = 0;

    m_arm.setControl(m_smoothArmMovement);
  }

  //Spins "Pizzabox" motor: velocity in rotations/sec and acceleration in rotations/sec^2
  public void spinPizzaBoxMotor(double velocity, double acceleration){
    VelocityVoltage spinPizzaBoxMotorRequest = new VelocityVoltage(velocity, acceleration, false, 0, 0, false, false, false);
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
  //Testing
  //return m_arm.getPosition().getValue() * (-100./36.);
    return m_arm.getPosition().getValue()/-Constants.Arm.GEAR_RATIO * Constants.Arm.ROTATION_TO_DEGREES;

 }

 /* The pizza box is the motor on the holding container on the arm*/ 
 public double getPizzaBoxAngle(){
  return m_pizzaBox.getPosition().getValue() * Constants.Arm.ROTATION_TO_DEGREES;
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getAbsolutePosition()*Constants.Arm.ROTATION_TO_DEGREES - Constants.Arm.ENCODER_OFFSET;
  }

  //Resets encoder angle to 0
  public void resetEncoderAngle()
  {
    m_encoder.reset();
  }

  @Override
  public void useOutput(double output, State setPoint)
  {
    //Comment out for testing purposes
    spinArm(output);
    //System.out.println("Target Speed is " + (output));
  }

  @Override
  public double getMeasurement()
  {
    return encoderGetAngle();
  }
}