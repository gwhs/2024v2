// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;





public class Climbsubsystem extends SubsystemBase {
  /** Creates a new Climbsubsystem. */
  private TalonFX climberArmLeft;
  private TalonFX climberArmRight;

  public Climbsubsystem(int motorIDLeft, int motorIDRight, boolean invertedLeft, boolean invertedRight) {

    this.climberArmLeft = new TalonFX(motorIDRight);
    this.climberArmRight = new TalonFX(motorIDRight);
    //for inversions
    climberArmLeft.setInverted(invertedLeft);
    climberArmRight.setInverted(invertedRight);

    climberArmLeft.setNeutralMode(NeutralModeValue.Brake);
    climberArmRight.setNeutralMode(NeutralModeValue.Brake);

    this.setZero();
  } 

  public double inchesToTicks(double inches){
    return inches * ClimbConstants.CLIMBER_RATIO * ClimbConstants.TICKS_PER_REVOLUTION / (ClimbConstants.PITCH_DIAMETER_30 * Math.PI);
  }

  public double ticksToInches(double d){
    return d / ClimbConstants.CLIMBER_RATIO / ClimbConstants.TICKS_PER_REVOLUTION * (ClimbConstants.PITCH_DIAMETER_30* Math.PI); 
  }

  public void setSpeed(double speed){ // not sure about this
    climberArmLeft.setControl(new VelocityTorqueCurrentFOC(speed));
    climberArmRight.setControl(new VelocityTorqueCurrentFOC(speed));
  }

  public void setBrake(){
    climberArmLeft.setNeutralMode(NeutralModeValue.Brake);
    climberArmRight.setNeutralMode(NeutralModeValue.Brake);
    
  }

  public void setZero(){
    climberArmLeft.setPosition(0);
    climberArmRight.setPosition(0);

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

