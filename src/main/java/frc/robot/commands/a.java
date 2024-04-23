// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class a extends InstantCommand {
  private boolean supplier;
  private String t;
  private Supplier<String> s;
  
  public a(Supplier<String> str) {
    s = str;
    supplier = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public a(String str) {
    t = str;
    supplier = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (supplier)
    sSubsystem.start(s.get());
    else 
      sSubsystem.start(t);
  }
}
