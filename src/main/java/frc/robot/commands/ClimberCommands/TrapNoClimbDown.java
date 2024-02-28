// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TrapNoClimbDown extends SequentialCommandGroup {

  //private ArmSubsystem armsubsystem;

  /** Creates a new Trap. */
  
  public TrapNoClimbDown(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ReactionSubsystem r) {

    addCommands(
          /*new align(), */
          //new arm move
          new ClimbUp(c, s, a, r), 
          /*new shoot(),*/ 
          //move arm back
    );
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

}
