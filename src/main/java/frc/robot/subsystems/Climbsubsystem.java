// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;




public class Climbsubsystem extends SubsystemBase {
  /** Creates a new Climbsubsystem. */
  private TalonFX climberArmLeft;
  private TalonFX climberArmRight;

  private final double CLIMBER_PID_KP = 1.9;
  private final double CLIMBER_PID_KI = 0;
  private final double CLIMBER_PID_KD = 0;
  private Constraints constraints = new Constraints(180.0, 300.0);

  private ProfiledPIDController leftPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 
  private ProfiledPIDController rightPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 

  private boolean upCheck = false;
  private boolean downCheck = false;

  DigitalInput bottomLeft = new DigitalInput(ClimbConstants.BOT_LEFT_LIMIT_ID);
  DigitalInput bottomRight = new DigitalInput(ClimbConstants.BOT_RIGHT_LIMIT_ID);
  DigitalInput topLeft = new DigitalInput(ClimbConstants.TOP_LEFT_LIMIT_ID);
  DigitalInput topRight = new DigitalInput(ClimbConstants.TOP_RIGHT_LIMIT_ID);

  //set canBus to the name of the canivore of the robot
  public Climbsubsystem(int motorIDLeft, int motorIDRight, boolean invertedLeft, boolean invertedRight, String canBus) {

    //makes the objects
    this.climberArmLeft = new TalonFX(motorIDLeft, canBus);
    this.climberArmRight = new TalonFX(motorIDRight, canBus);
    //sets inversion for each climb motor
    climberArmLeft.setInverted(invertedLeft);
    climberArmRight.setInverted(invertedRight);


    climberArmLeft.setNeutralMode(NeutralModeValue.Brake);
    climberArmRight.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
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
    StatusCode leftMotorStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode rightMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      leftMotorStatus = climberArmLeft.getConfigurator().apply(configs);
      rightMotorStatus = climberArmRight.getConfigurator().apply(configs);
      if (leftMotorStatus.isOK() && rightMotorStatus.isOK()) break;
    }
    if(!leftMotorStatus.isOK() || !rightMotorStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + leftMotorStatus.toString());
    }

    climberArmLeft.setControl(new NeutralOut());
    climberArmRight.setControl(new NeutralOut());

  } 

  public void upMotor() {
    upCheck = true;
    downCheck = false;
    leftPIDcontroller.setGoal(-198.94);
    rightPIDcontroller.setGoal(198.4);
  }

  public void downMotor() {
    downCheck = true;
    upCheck = false;
    leftPIDcontroller.setGoal(0.5);
    rightPIDcontroller.setGoal(0.5);
  }

  //makes the motor move
  public void setSpeed(double leftSpeed, double rightSpeed){ // speed should be in rotations per second
    climberArmLeft.setControl(new VelocityVoltage(-leftSpeed, 
                                                  50, // rotations per second^2
                                                  true, // if we bought it then set true and get more power
                                                  0, 
                                                  0,
                                                  false, 
                                                  false, 
                                                  false));
    climberArmRight.setControl(new VelocityVoltage(rightSpeed, 
                                                  50, // rotations per second^2
                                                  true, // if we bought it then set true and get more power
                                                  0, 
                                                  0, 
                                                  false, 
                                                  false, 
                                                  false));

    
    // climberArmRight.setControl(new VelocityTorqueCurrentFOC(rightSpeed, 5, 0, 0, false, false, false));
    // climberArmLeft.setControl(new VelocityTorqueCurrentFOC(leftSpeed, 5, 0, 0, false, false, false));
  }

  //makes motors stop spinning
  public void stopClimbLeft(){
    climberArmLeft.stopMotor();
    climberArmLeft.setControl(new NeutralOut());
  }

  public void stopClimbRight() {
    climberArmRight.stopMotor();
    climberArmRight.setControl(new NeutralOut());
  }

  //gets the position for left motor; return the amount of rotations
  public double getPositionLeft(){
    return climberArmLeft.getPosition().getValue();
  }

  //gets the position for right motor; return the amount of rotations
  public double getPositionRight() {
    return climberArmRight.getPosition().getValue();
  }

  public boolean getTopLeftLimit() {     
    return (!topLeft.get());
  }

  public boolean getTopRightLimit() {
    return (!topRight.get());
  }

  public boolean getBotLeftLimit() {
    return (!bottomLeft.get());
  }

  public boolean getBotRightLimit() {
    return (!bottomRight.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double leftPIDvalue = leftPIDcontroller.calculate(getPositionLeft());
    double rightPIDvalue = rightPIDcontroller.calculate(getPositionRight());
    //System.out.println("left: " + leftPIDvalue + "/n right: " + rightPIDvalue);
    if (upCheck && getTopLeftLimit()) {
      leftPIDvalue = 0;
    }
    else if (upCheck && getTopRightLimit()) {
      rightPIDvalue = 0;
    }

    if (downCheck && getBotLeftLimit()) {
      leftPIDvalue = 0;
    }
    else if (downCheck && getBotRightLimit()) {
      rightPIDvalue = 0;
    }

    if (upCheck) {
      setSpeed(-leftPIDvalue/4, rightPIDvalue/4);
    }

    if (downCheck) {
      setSpeed(-leftPIDvalue/4, rightPIDvalue/4);
    }

    


  }
}

