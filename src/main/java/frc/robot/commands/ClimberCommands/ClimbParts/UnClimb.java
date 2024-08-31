// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.ReactionSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnClimb extends SequentialCommandGroup {
  /** Creates a new UnClimb. */
  public UnClimb(Climbsubsystem c, ArmSubsystem a, ReactionSubsystem r) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(()->DataLogManager.log("Command Start: UnClimb")),
      new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ANGLE).withTimeout(0.5),
      Commands.waitUntil(()->a.checkEncoderAngleForClimb()),
      new MotorUp(c, a).withTimeout(5),
      new Retract(r).withTimeout(1),
      Commands.runOnce(()->DataLogManager.log("Command End: UnClimb"))
    );
  }
}
