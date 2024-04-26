// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.CTRETeleopDrive;

public class FaceSpeaker extends Command {
  /** Creates a new FaceSpeaker. */
  private final CTRETeleopDrive drive;
  public FaceSpeaker(CTRETeleopDrive drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.faceSpeaker = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.faceSpeaker = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
