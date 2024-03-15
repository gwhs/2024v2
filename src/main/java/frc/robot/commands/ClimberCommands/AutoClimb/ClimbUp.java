// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.AutoClimb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.*;

public class ClimbUp extends SequentialCommandGroup {

  /** Creates a new ClimbUp.*/
  
  public ClimbUp(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ReactionSubsystem r) {
    addCommands(
          new ParallelCommandGroup(new SpinToArmAngle(a, 135)/* , new Extend(r)*/),
                  new SequentialCommandGroup(new WaitCommand(0.5), new MotorUp(c)),
          new DriveForClimb(s, -0.4),
          new MotorDown(c)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.

  }
}
