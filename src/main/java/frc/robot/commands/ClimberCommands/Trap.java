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

public class Trap extends SequentialCommandGroup {

  //private ArmSubsystem armsubsystem;

  /** Creates a new Trap. */
  
  public Trap(Climbsubsystem c, SwerveSubsystem s/* , ArmSubsystem a*/ ) {

    addCommands(
          /*new align(), */
          new ClimbUp(c, s), 
          /*new shoot(),*/ 
          new ClimbDown(c, s)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

}