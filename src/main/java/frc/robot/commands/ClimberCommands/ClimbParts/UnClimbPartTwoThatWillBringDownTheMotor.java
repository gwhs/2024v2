// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.Arm.SwingBackServoTheSecond;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.ArmSubsystem.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnClimbPartTwoThatWillBringDownTheMotor extends SequentialCommandGroup {
  /** Creates a new UnClimbPartTwoThatWillBringDownTheMotor. */
  public UnClimbPartTwoThatWillBringDownTheMotor(Climbsubsystem c, ArmSubsystem a, ReactionSubsystem r, PizzaBoxSubsystem p) {

    addCommands(
      Commands.runOnce(()->DataLogManager.log("Command Start: UnClimbPartTwo")),
      new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ANGLE).withTimeout(0.5),
      Commands.waitUntil(()->a.checkEncoderAngleForClimb()),
      new MotorDown(c, a).withTimeout(3),
      new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ARNGLE_FOR_SERVO).withTimeout(2.5),
      new SwingBackServoTheSecond(p),
      new WaitCommand(0.7),
      new SpinToArmAngle(a, Arm.INTAKE_ANGLE).withTimeout(.1),
      Commands.runOnce(()->DataLogManager.log("Command End: UnClimbPartTwo"))
      );
  }
}
