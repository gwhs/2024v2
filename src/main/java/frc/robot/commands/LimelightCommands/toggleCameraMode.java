// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;

public class toggleCameraMode extends Command {
  /** Creates a new toggleLimelightPoseEstimation. */
  LimeLightSub l;
  LimeLightSub limeSub;
  public toggleCameraMode(LimeLightSub l) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.l = l;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l.cameraMode = !l.cameraMode;
    if(l.cameraMode) {
      LimelightHelpers.setStreamMode_PiPMain("limelight");
    }
    else {
      LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
