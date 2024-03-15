// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ClimbParts.*;
import frc.robot.commands.ReactionArmCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.*;

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
      new UnClimb(c), new WaitCommand(2),
      new UnClimbPartTwoThatWillBringDownTheMotor(c, r), new WaitCommand(2),
      // new PickUpFromGroundAndPassToPizzaBox(p, a, i), new WaitCommand(2),
      // new PickUpFromGroundAndPassToPizzaBox(p, a, i), new WaitCommand(2),
      // new ScoreInAmp(p, a), new WaitCommand(2),
      new ScoreInSpeakerUnderHand(p, a), new WaitCommand(2)

    );
  }
}
