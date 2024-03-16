// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Arm.ScoreInAmp;
import frc.robot.commands.Arm.ScoreInSpeakerUnderHand;
import frc.robot.commands.ClimberCommands.ClimbParts.ClimbAndShoot;
import frc.robot.commands.ClimberCommands.ClimbParts.PrepClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.UnClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.UnClimbPartTwoThatWillBringDownTheMotor;
import frc.robot.commands.IntakeCommands.PickUpFromGroundAndPassToPizzaBox;
import frc.robot.commands.ReactionArmCommands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem.PizzaBox;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCheck extends SequentialCommandGroup {
  /** Creates a new SystemCheck. */
  public SystemCheck(ArmSubsystem a, Climbsubsystem c, IntakeSubsystem i, PizzaBoxSubsystem p, ReactionSubsystem r, SwerveSubsystem s) {
    
    addCommands(
      //TO-DO: Add condition for different systemcheck if arm/intake is broken
      new Extend(r), new WaitCommand(2),
      new Retract(r), new WaitCommand(2),
      new PrepClimb(c, s, a, r), new WaitCommand(2),
      new ClimbAndShoot(c, s, a, p, r), new WaitCommand(2),
      new UnClimb(c, a, r), new WaitCommand(2),
      new UnClimbPartTwoThatWillBringDownTheMotor(c, s, a, r), new WaitCommand(2),
      new PickUpFromGroundAndPassToPizzaBox(p, a, i), new WaitCommand(2),
      new PickUpFromGroundAndPassToPizzaBox(p, a, i), new WaitCommand(2),
      new ScoreInAmp(p, a), new WaitCommand(2),
      new ScoreInSpeakerUnderHand(p, a), new WaitCommand(2)

    );
  }
}
