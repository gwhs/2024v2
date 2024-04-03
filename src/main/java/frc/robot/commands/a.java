// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BestSubsystem;

public class a extends Command {
  /** Creates a new a. */
  private double initTimer = 0;
  private double timer = 0;
  private String s;
  public a(String str) {
    s = str;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BestSubsystem.start(s);
    initTimer = Timer.getFPGATimestamp(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    timer = Timer.getFPGATimestamp(); 
    return (initTimer + 1 > timer);
  }
}
