// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;





public class Climbsubsystem extends SubsystemBase {
  /** Creates a new Climbsubsystem. */
  private TalonFX climberArmLeft;
  private TalonFX climberArmRight;
  //set canBus to the name of the canivore of the robot
  public Climbsubsystem(int motorIDLeft, int motorIDRight, boolean invertedLeft, boolean invertedRight, String canBus) {

    this.climberArmLeft = new TalonFX(motorIDLeft, canBus);
    this.climberArmRight = new TalonFX(motorIDRight, canBus);
    //for inversions
    climberArmLeft.setInverted(invertedLeft);
    climberArmRight.setInverted(invertedRight);

    climberArmLeft.setNeutralMode(NeutralModeValue.Brake);
    climberArmRight.setNeutralMode(NeutralModeValue.Brake);

  } 

  public double inchesToTicks(double inches){
    return inches * ClimbConstants.CLIMBER_RATIO * ClimbConstants.TICKS_PER_REVOLUTION / (ClimbConstants.PITCH_DIAMETER_30 * Math.PI);
  }

  public double ticksToInches(double d){
    return d / ClimbConstants.CLIMBER_RATIO / ClimbConstants.TICKS_PER_REVOLUTION * (ClimbConstants.PITCH_DIAMETER_30* Math.PI); 
  }

  public void setSpeed(double speed){ // change to foc later
    double leftSpeed = speed;
    double rightSpeed = speed;
    climberArmLeft.setControl(new VelocityVoltage(leftSpeed, 
                                                  5, 
                                                  false, 
                                                  0, 
                                                  0,
                                                  false, 
                                                  false, 
                                                  false));
    climberArmRight.setControl(new VelocityVoltage(rightSpeed, 
                                                  5, 
                                                  false, 
                                                  0, 
                                                  0, 
                                                  false, 
                                                  false, 
                                                  false));
  }

  public void stopClimb(){
    climberArmLeft.stopMotor();
    climberArmRight.stopMotor();
    
  }

  public double getPositionLeft(){
    return climberArmLeft.getPosition().getValue();
  }

  public double getPositionRight() {
    return climberArmRight.getPosition().getValue();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

