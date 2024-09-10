// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem;

public interface IntakeIO {
  public double getIntakeArmAngle();
  public void setArmSpeed(double speed);
  public void setSpinSpeed(double speed);
  public boolean getNoteSensor();
  public double getSpinSpeed();
  public void update();
}
