// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Trap extends Command {

  private Climbsubsystem climbsubsystem;
  private SwerveSubsystem swerve;
  private ArmSubsystem armsubsystem;

  /** Creates a new Trap. */
  
  public Trap(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ) {
    climbsubsystem = c;
    swerve = s;
    armsubsystem = a;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbsubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                                              new ParallelCommandGroup(new limelightsuff(), new armstuff(), new ClimbDown(climbsubsystem, swerve)), 
                                              new ParallelCommandGroup(new ClimbUp(climbsubsystem, swerve)),
                                              new armshootstuff(),
                                              new ParallelCommandGroup( new armstuff(), new ClimbDown(climbsubsystem, swerve)));
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
    return false;
  }
}
