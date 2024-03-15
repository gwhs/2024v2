// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PrepClimb extends SequentialCommandGroup {
  /** Creates a new PrepClimb. */
  public PrepClimb(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ReactionSubsystem r) {
    addCommands(
      new PrintCommand("prepclimb initialize"),
      new SpinToArmAngle(a, 260).withTimeout(1),
      Commands.waitUntil(()->a.checkEncoderAngleForClimb()), 
      new MotorUp(c, s).withTimeout(5),
      new PrintCommand("prepclimb finished")
    );
  
    // Use addRequirements() here to declare subsystem dependencies.
    
  }
}
