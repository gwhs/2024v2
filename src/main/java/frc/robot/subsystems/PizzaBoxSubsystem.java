// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration; 

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Servo;



public class PizzaBoxSubsystem extends SubsystemBase {

  public static final class PizzaBox {

    public static final int PIZZABOX_ID = 23;
    public static final int SERVO_PWN_SLOT = 0;
   
  }

  private TalonFX m_pizzaBox;
  private Servo m_servo;

  public PizzaBoxSubsystem(int pizzaBoxId, String pizzaBoxCanbus, int channelServo)
  {

      m_pizzaBox = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
      m_servo = new Servo(channelServo);
      
      

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
}