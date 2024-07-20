// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.Arm.SwingForwardServoTheSecond;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem.PizzaBox;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PrepClimb extends SequentialCommandGroup {
  /** Creates a new PrepClimb. */
  public PrepClimb(Climbsubsystem c, ArmSubsystem a, ReactionSubsystem r, PizzaBoxSubsystem p) {
    addCommands(
      Commands.runOnce(()->DataLogManager.log("Command Start: PrepClimb")),
      new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ARNGLE_FOR_SERVO).withTimeout(4),
      new SwingForwardServoTheSecond(p),
      new WaitCommand(0.7),
      new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ANGLE).withTimeout(1),
      Commands.waitUntil(()->a.checkEncoderAngleForClimb()), 
      new MotorUp(c, a).withTimeout(5),
      Commands.runOnce(()->DataLogManager.log("Command End: PrepClimb"))
    );
  }
}
