// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BestSubsystem extends SubsystemBase {

  private static Orchestra orchestra = new Orchestra();

  /** Creates a new BestSubsystem. */
  public BestSubsystem() {
    
  }

  public static void join(TalonFX motor) {
    orchestra.addInstrument(motor);
  }

  public static void start(String title) {
    orchestra.stop();
    orchestra.loadMusic(title);
    orchestra.play();
  }

  public static void end() {
    orchestra.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
