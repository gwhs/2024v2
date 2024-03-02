// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SetupShuffleboard extends SubsystemBase {
  /** Creates a new SetupShuffleboard. */
  public SetupShuffleboard() {
  }

  public static void setupShuffleboard(){
    Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5801").withSize(4,2).withPosition(5, 0);
    Shuffleboard.getTab("GameTab").putNumber();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
