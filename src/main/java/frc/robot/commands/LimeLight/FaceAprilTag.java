// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FaceAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem driSwerveSubsystem;
  private final LimeLightSub limeLightSub;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FaceAprilTag(SwerveSubsystem driSwerveSubsystem, LimeLightSub limeLightSub) {
    this.driSwerveSubsystem = driSwerveSubsystem;
    this.limeLightSub = limeLightSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driSwerveSubsystem, limeLightSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driSwerveSubsystem.drive((0), limeLightSub.getTx(), false);

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