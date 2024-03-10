// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Util.UtilMotor;




public class Climbsubsystem extends SubsystemBase {
  /** Creates a new Climbsubsystem. */
  private TalonFX climberArmLeft;
  private TalonFX climberArmRight;

  private final double CLIMBER_PID_KP = 2.5;
  private final double CLIMBER_PID_KI = 0;
  private final double CLIMBER_PID_KD = 0;
  private Constraints constraints = new Constraints(180.0, 300.0);

  private ProfiledPIDController leftPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 
  private ProfiledPIDController rightPIDcontroller = new ProfiledPIDController(CLIMBER_PID_KP, CLIMBER_PID_KI, CLIMBER_PID_KD, constraints); 

  private double leftGoalDistance = 193.94;
  private double rightGoalDistance = 193.4;

  private boolean checkForUp = false;
  private boolean emergencyStop = false;

  DigitalInput bottomLeft = new DigitalInput(ClimbConstants.BOT_LEFT_LIMIT_ID);
  DigitalInput bottomRight = new DigitalInput(ClimbConstants.BOT_RIGHT_LIMIT_ID);
  DigitalInput topLeft = new DigitalInput(ClimbConstants.TOP_LEFT_LIMIT_ID);
  DigitalInput topRight = new DigitalInput(ClimbConstants.TOP_RIGHT_LIMIT_ID);

  private final StaticBrake brake;

  //set canBus to the name of the canivore of the robot
  public Climbsubsystem(int motorIDLeft, int motorIDRight, boolean invertedLeft, boolean invertedRight, String canBus) {

    //makes the objects
    this.climberArmLeft = new TalonFX(motorIDLeft, canBus);
    this.climberArmRight = new TalonFX(motorIDRight, canBus);
    //sets inversion for each climb motor
    climberArmLeft.setInverted(invertedLeft);
    climberArmRight.setInverted(invertedRight);

    brake = new StaticBrake();
    
    UtilMotor.configMotor(climberArmLeft, 0.11, 0.5, 0.0001, 0.12, 12, 40, true);
    UtilMotor.configMotor(climberArmRight, 0.11, 0.5, 0.0001, 0.12, 12, 40, true);

    leftPIDcontroller.setGoal(getPositionLeft());
    rightPIDcontroller.setGoal(getPositionRight());
  } 


  //makes the motor move
  public void setSpeed(double leftSpeed, double rightSpeed){ // speed should be in rotations per second
    if(emergencyStop) {
      leftSpeed = 0;
      rightSpeed = 0;
    }

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

  }

  //makes motors stop spinning
  public void stopClimbLeft(){
    climberArmLeft.stopMotor();
    climberArmLeft.setControl(brake);
  }

  public void stopClimbRight() {
    climberArmRight.stopMotor();
    climberArmLeft.setControl(brake);
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

  public void setSetGoal(double left, double right) {
    leftGoalDistance = left;
    rightGoalDistance = right;
  } 

  public void stopClimbMotorInCaseOfEmergencySoThisWillStopTheClimbNoMatterIfItIsGoingUpOrDown() {
    if(!emergencyStop) {
      emergencyStop = true;
      leftPIDcontroller.setGoal(getPositionLeft());
      rightPIDcontroller.setGoal(getPositionRight());
    }
    else{
      emergencyStop = false;
    }
  }

  public void upMotor() {
    checkForUp = true;
    leftPIDcontroller.setGoal(-leftGoalDistance);
    rightPIDcontroller.setGoal(rightGoalDistance);
  }

  public void downMotor() {
    checkForUp = false;
    leftPIDcontroller.setGoal(-.3); //-0.3
    rightPIDcontroller.setGoal(.3); //-0.3
  }

  public void downMotorHarmony() {
    checkForUp = false;
    
  }

  @Override
  public void periodic() {
    double leftPIDvalue = leftPIDcontroller.calculate(getPositionLeft());
    double rightPIDvalue = rightPIDcontroller.calculate(getPositionRight());

    if (checkForUp && getTopLeftLimit()) {
      leftPIDvalue = 0;
    }
    if (checkForUp && getTopRightLimit()) {
      rightPIDvalue = 0;
    }

    if (!checkForUp && getBotLeftLimit()) {
      leftPIDvalue = 0;
    }
    if (!checkForUp && getBotRightLimit()) {
      rightPIDvalue = 0;
    }
    
    setSpeed(-leftPIDvalue, rightPIDvalue);
  }
}

