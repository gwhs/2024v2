// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem.PizzaBoxSubsystem;

public class LockHeadingToSourceForIntake extends Command {
  /** Creates a new LockHeadingToSourceForIntake. */
  public final CTRETeleopDrive drive;
  public final ArmSubsystem armSubsystem;
  public final PizzaBoxSubsystem pizzaBoxSubsystem;
  public LockHeadingToSourceForIntake(CTRETeleopDrive drive, ArmSubsystem armSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.armSubsystem = armSubsystem;
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isHeadingLock = true;
    pizzaBoxSubsystem.slurp_command();
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
    pizzaBoxSubsystem.stopMotor();
    armSubsystem.targetArmAngle(90);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
