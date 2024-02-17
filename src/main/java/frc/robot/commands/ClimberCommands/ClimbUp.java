// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbUp extends Command {

  private Climbsubsystem climbersubsystem;
  private SwerveSubsystem swerve;

  /** Creates a new ClimbUp. */
  public ClimbUp(Climbsubsystem c, SwerveSubsystem s) {
    climbersubsystem = c;
    swerve = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbersubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup(
      new ParallelCommandGroup(/*new alignment(), new movearm(), */ 
        new SequentialCommandGroup(new WaitCommand(0), new MotorUp(climbersubsystem, swerve))),
      /*new ParallelCommandGroup(new driveforward(), new Reactionbar()),*/
      new MotorDown(climbersubsystem, swerve)
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbersubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
