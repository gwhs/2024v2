// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.FaceSpeaker;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.testcontainers.DriveContainer;

public class SwitchDrivebase extends Command {
  private final SwerveSubsystem m_Subsystem;
  private final FaceSpeaker drivebase;
  /** Creates a new SwitchDrivebase. */
  public SwitchDrivebase(FaceSpeaker speaker,SwerveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = subsystem;
    drivebase = speaker;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsystem.setDefaultCommand(drivebase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
