// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class UpperArmIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;
  private double armPositionAngle;

  public UpperArmIntake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // moving arm motor back up to original position (0)
  @Override
  public void execute() {
      intakeSubsystem.setArmAngle(0);
      System.out.println(intakeSubsystem.encoderGetAngle()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    armPositionAngle = intakeSubsystem.getArmPos();
    return Math.abs(armPositionAngle - 0) < Constants.IntakeConstants.TOLERANCE;
  }

}
