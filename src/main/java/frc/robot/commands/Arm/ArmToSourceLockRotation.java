// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveCommands.LockHeadingToSourceForIntake;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToSourceLockRotation extends SequentialCommandGroup {
  /** Creates a new ArmToSourceLockRotation. */
  public ArmToSourceLockRotation(TeleopDrive drive, ArmSubsystem armSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LockHeadingToSourceForIntake(null),
      new SpinToArmAngle(armSubsystem, 170),
      new SpinPizzaBoxSource(pizzaBoxSubsystem)

    );
  }
}
