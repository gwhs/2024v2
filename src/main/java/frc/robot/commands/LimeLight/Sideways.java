// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Sideways extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem driSwerveSubsystem;
  private final LimeLightSub limeLightSub;

  private double distance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Sideways(SwerveSubsystem driSwerveSubsystem, LimeLightSub limeLightSub) {
    this.driSwerveSubsystem = driSwerveSubsystem;
    this.limeLightSub = limeLightSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driSwerveSubsystem, limeLightSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightSub.setPoint(0, "y");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = limeLightSub.getErrorY(); 
    driSwerveSubsystem.drive(new Translation2d(0, distance), 0, true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (limeLightSub.getDistanceY() < 0.03 && limeLightSub.getDistanceY() > -0.03);
    // return false;
  }
}
