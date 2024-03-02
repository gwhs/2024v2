// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetupShuffleboard extends SubsystemBase {
  /** Creates a new SetupShuffleboard. */
  public SetupShuffleboard() {
  }

  public static void setupShuffleboard(SwerveSubsystem swerve){
    Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5801").withSize(4,2).withPosition(5, 0);
    Shuffleboard.getTab("GameTab").add("Filed", swerve.getField2d());
    autoChooser = AutoBuilder.buildAutoChooser("0-S(Amp)-0");
    Shuffleboard.getTab("Autonomous").add("Autonomous Chooser", autoChooser).withSize(2, 1);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
