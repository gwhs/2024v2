// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbUp extends SequentialCommandGroup {

  /** Creates a new ClimbUp.*/
  public ClimbUp(Climbsubsystem c, SwerveSubsystem s) {
    addCommands(
          new ParallelCommandGroup(/*new movearm(), */ 
              new SequentialCommandGroup(new WaitCommand(2), new MotorUp(c, s))),
          /*new ParallelCommandGroup(new driveforward(), new Reactionbar()),*/
          new MotorDown(c, s)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.

  }
}
