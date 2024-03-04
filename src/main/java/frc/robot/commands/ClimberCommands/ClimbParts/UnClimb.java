// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnClimb extends SequentialCommandGroup {
  /** Creates a new UnClimb. */
  public UnClimb(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, PizzaBoxSubsystem p, ReactionSubsystem r) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (true || Math.abs(a.encoderGetAngle() - 135) <= 5) 
      addCommands(
        new MotorUp(c, s),
        new ParallelCommandGroup(new WaitCommand(4), new Retract(r).withTimeout(0.5))
        //,new MotorDown(c, s)
      );
  }
}
