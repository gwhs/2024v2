// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.CTRETeleopDrive;

public class DecreaseSpeed extends Command {
  public final CTRETeleopDrive drive;
  /** Creates a new decreaseSpeed. */
  public DecreaseSpeed(CTRETeleopDrive drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isSlow = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.isSlow = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
