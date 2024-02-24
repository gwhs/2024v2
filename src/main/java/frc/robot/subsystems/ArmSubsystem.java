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

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
//
import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//


public class ArmSubsystem extends ProfiledPIDSubsystem {

  public static final class Arm {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int ARM_MAX_ANGLE = 300;
    public static final int ARM_MIN_ANGLE = 0;
    public static final int ROTATION_TO_DEGREES = 360;
    public static final double GEAR_RATIO = 118.587767088;
    public static final double ENCODER_RAW_TO_ROTATION = 8132.;
    public static final double ENCODER_OFFSET = -12.224; 
    public static final int ARM_ID = 18;
    //
    public static final double KSVOLTS = 0; 
    public static final double KGVOLTS = 0;
    //
    //Arm ID Jalen Tolbert
    public static final int PIZZABOX_ID = 23;
    public static final int SERVO_PWN_SLOT = 0;
    public static final int ENCODER_DIO_SLOT = 0;
    public static final int AMP_ANGLE = 0;
    public static final int TRAP_ANGLE = 0;
    public static final int SPEAKER_LOW_ANGLE = 100;
    public static final int SPEAKER_HIGH_ANGLE = 204;
    public static final int INTAKE_ANGLE = 64;
    public static final int CLIMBING_ANGLE = 0;
  }

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private TalonFX m_pizzaBox;
  private Servo m_servo;
  private ArmFeedforward armFeedForward;

  public ArmSubsystem(int armId, String armCanbus, int pizzaBoxId, String pizzaBoxCanbus, int channel1, int channelServo)
  {
    super(new ProfiledPIDController(5, .1, 0, new Constraints(2*Math.PI, 10)));
    getController().setTolerance(2 * (Math.PI/180));
    //TrapezoidProfile either velocity or position
      m_arm = new TalonFX(armId, armCanbus);
      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_encoder = new DutyCycleEncoder(channel1);
      m_servo = new Servo(channelServo);
      armFeedForward = new ArmFeedforward(Arm.KSVOLTS, Arm.KGVOLTS, 0, 0);
      
      

      // Needed? PIDController ArmPID = new PIDController(0, 0, 0);


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
      for (int i = 0; i < 5; ++i) {
        motorStatus = m_pizzaBox.getConfigurator().apply(configs);
        if (motorStatus .isOK()) break;
      }
      if(!motorStatus.isOK()) {
        System.out.println("Could not apply configs, error code: " + motorStatus.toString());
      }

    Shuffleboard.getTab("Arm").addDouble("Encoder Angle", ()->encoderGetAngle()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3)
    .withPosition(0, 0);
    Shuffleboard.getTab("Arm").addDouble("Goal in degrees", ()->getController().getGoal().position * (180/Math.PI));
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
  if(speed < -15) { //Will not be less than minimum angle
    speed = -15;
  }
  else if (speed > 15) { // Will not be greater than maximum angle
    speed = 15;
  }
  VoltageOut armSpinRequest = new VoltageOut(-speed, true, false, false, false);
  m_arm.setControl(armSpinRequest);
 }

 public void targetArmAngle(double angle)
 {
  double calculatedAng = angle ;
  if(calculatedAng  < Arm.ARM_MIN_ANGLE) { //Will not be less than minimum angle
    calculatedAng = Arm.ARM_MIN_ANGLE;
  }
  else if (calculatedAng > Arm.ARM_MAX_ANGLE ) { // Will not be greater than maximum angle
    calculatedAng = Arm.ARM_MAX_ANGLE;
  }

  System.out.println("TargetArmAngle works, angle value : " + calculatedAng * Math.PI/180);
  setGoal(calculatedAng * Math.PI/180);
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
    return m_arm.getPosition().getValue()/-Arm.GEAR_RATIO * Arm.ROTATION_TO_DEGREES;
 }

 /* The pizza box is the motor on the holding container on the arm*/ 
 public double getPizzaBoxAngle(){
  return m_pizzaBox.getPosition().getValue() * Arm.ROTATION_TO_DEGREES;
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getAbsolutePosition()*Arm.ROTATION_TO_DEGREES - Arm.ENCODER_OFFSET;
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
    double feedForward = armFeedForward.calculate(setPoint.position, setPoint.velocity);
    SmartDashboard.putNumber("feedForward calculation", feedForward);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("setPoint position", setPoint.position);
    SmartDashboard.putNumber("setPoint velocity", setPoint.velocity);
    spinArm(output + feedForward);
    //System.out.println("Target Speed is " + (output));
  }

  @Override
  public double getMeasurement()
  {
    return encoderGetAngle() * Math.PI/180;
  }
}