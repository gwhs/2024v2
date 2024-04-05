// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class StraightenWheelCommand extends Command {
  /** Creates a new StraightenWheelCommand. */
  private final SwerveSubsystem m_Subsystem;
  public StraightenWheelCommand(SwerveSubsystem m_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = m_Subsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsystem.drive(new Translation2d(1, 0), 0, false);
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
    return true;
  }
}