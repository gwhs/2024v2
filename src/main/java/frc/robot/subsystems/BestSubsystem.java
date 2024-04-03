// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BestSubsystem extends SubsystemBase {

  private static Collection<ParentDevice> whole = new ArrayList<ParentDevice>();
  private static Orchestra orchestra;

  /** Creates a new BestSubsystem. */
  public BestSubsystem() {
    orchestra = new Orchestra(whole);
  }

  public static void join(TalonFX motor) {
    whole.add(motor);
  }

  public static void start() {
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
