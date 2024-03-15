// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.*;
import frc.robot.commands.ReactionArmCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.*;

public class PrepClimb extends SequentialCommandGroup {
  /** Creates a new PrepClimb. */
  public PrepClimb(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ReactionSubsystem r) {
    addCommands(
      new PrintCommand("prepclimb initialize"),
      // new SpinToArmAngle(a, 135).withTimeout(3),
      // Commands.waitUntil(()->a.checkEncoderAngleForClimb()),
      new ParallelCommandGroup(new Extend(r).withTimeout(0.5), 
                               new MotorUp(c).withTimeout(5)),
      new PrintCommand("prepclimb finished")
    );
  
    // Use addRequirements() here to declare subsystem dependencies.
    
  }
}
