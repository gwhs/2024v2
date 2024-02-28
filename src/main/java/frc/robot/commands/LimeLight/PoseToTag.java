// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class PoseToTag extends Command {
  private final LimeLightSub limeLightSub;
  private Translation2d targetRobotPos;
  /** Creates a new PoseToTag. */
  public PoseToTag(LimeLightSub m_LightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    limeLightSub = m_LightSub;
    addRequirements(m_LightSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(limeLightSub.getID() > Double.NEGATIVE_INFINITY)
    {
      targetRobotPos = new Translation2d(, );
    }

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
    return false;
  }
}
