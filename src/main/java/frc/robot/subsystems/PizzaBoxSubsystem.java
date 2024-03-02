// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.UtilMotor;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration; 

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class PizzaBoxSubsystem extends SubsystemBase {

  public static final class PizzaBox {

    public static final int PIZZABOX_ID = 23;
    public static final int SERVO_PWN_SLOT = 0;
    public static final int START_SPIN_DEGREE = 100;
  }

  private TalonFX m_pizzaBox;
  private Servo m_servo;

  public PizzaBoxSubsystem(int pizzaBoxId, String pizzaBoxCanbus, int channelServo)
  {

      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_servo = new Servo(channelServo);
    
    UtilMotor.configMotor(m_pizzaBox, .11, 0, 0, .12, 15, 50, false);      
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

  //Stops pizzaBox motor
  public void stopPizzaBoxMotor() {
    m_pizzaBox.stopMotor();
  }

  public boolean isAtVelocity(double vel){
    System.out.println("getVelocity value is " + m_pizzaBox.getVelocity().getValue());
    System.out.println("getVelocity value is " + m_pizzaBox.getVelocity().getValue());
    System.out.println("getVelocity value is " + m_pizzaBox.getVelocity().getValue());

    return m_pizzaBox.getRotorVelocity().getValue() >= vel;
  }
}