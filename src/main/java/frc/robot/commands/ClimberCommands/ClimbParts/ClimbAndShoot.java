// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAndShoot extends SequentialCommandGroup {
  /** Creates a new ClimbAndShoot. */
  public ClimbAndShoot(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, PizzaBoxSubsystem p, ReactionSubsystem r) {
    // Add your commands in the addCommands() call, e.g.
    addCommands (
      new PrintCommand("climb and shoot initialize"),
      new SpinToArmAngle(a, 260).withTimeout(1),
      Commands.waitUntil(()->a.checkEncoderAngleForClimb()),
      new MotorDown(c).alongWith(new Extend(r)).withTimeout(3),
      new ScoreInTrapStutter(p, a),
      new PrintCommand("climb and shoot finished")
      );

    
  }
}
