// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;

public class LockHeadingToSourceForIntake extends Command {
  /** Creates a new LockHeadingToSourceForIntake. */
  public final TeleopDrive drive;
  public final ArmSubsystem armSubsystem;
  public final PizzaBoxSubsystem pizzaBoxSubsystem;
  public LockHeadingToSourceForIntake(TeleopDrive drive, ArmSubsystem armSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.armSubsystem = armSubsystem;
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isHeadingLock = true;
    pizzaBoxSubsystem.spinPizzaBoxMotor(-50, 100);
    armSubsystem.targetArmAngle(Constants.IntakeConstants.SOURCE_INTAKE_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.isHeadingLock = false;
    pizzaBoxSubsystem.stopPizzaBoxMotor();
    armSubsystem.targetArmAngle(90);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
