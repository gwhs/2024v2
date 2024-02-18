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

public class ClimbDown extends Command {

  private Climbsubsystem climbersubsystem;
  private SwerveSubsystem swerve;

  /** Creates a new ClimbDown. */
  public ClimbDown(Climbsubsystem c, SwerveSubsystem s) {
    climbersubsystem = c;
    swerve = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup(new MotorUp(climbersubsystem, swerve),
      /*new ParallelCommandGroup(new movebackwards(), new Reactionbar())*/
      new MotorDown(climbersubsystem, swerve) /*, new armstuff() */ 
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END!!!");
    climbersubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
