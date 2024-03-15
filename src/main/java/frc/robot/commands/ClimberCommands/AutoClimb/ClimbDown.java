// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.AutoClimb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.*;

public class ClimbDown extends SequentialCommandGroup {

  /** Creates a new ClimbDown. */
  public ClimbDown(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a) {

    addCommands(
          new MotorUp(c),
          new ParallelCommandGroup(new DriveForClimb(s, 0.4)/* , new Retract(r)*/),
          new MotorDown(c)
    );

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

}
