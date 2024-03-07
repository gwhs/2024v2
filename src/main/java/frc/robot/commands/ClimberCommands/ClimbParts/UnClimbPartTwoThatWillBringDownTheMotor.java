// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnClimbPartTwoThatWillBringDownTheMotor extends SequentialCommandGroup {
  /** Creates a new UnClimbPartTwoThatWillBringDownTheMotor. */
  public UnClimbPartTwoThatWillBringDownTheMotor(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinToArmAngle(a, 135).withTimeout(1),
      Commands.waitUntil(()->Math.abs(a.encoderGetAngle() - 130) <= 5),
      new MotorDown(c, s).withTimeout(5));
  }
}
