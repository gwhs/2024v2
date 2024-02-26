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

public class ClimbDown extends SequentialCommandGroup {

  /** Creates a new ClimbDown. */
  public ClimbDown(Climbsubsystem c, SwerveSubsystem s) {

    addCommands(
          new MotorUp(c, s),
          /*new ParallelCommandGroup(new movebackwards(), new Reactionbar())*/
          new MotorDown(c, s) /*, new armstuff() */ 
    );

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

}
