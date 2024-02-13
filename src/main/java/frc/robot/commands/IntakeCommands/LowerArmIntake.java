// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LowerArmIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSubsystem;

  private double armPositionAngle;
  private double angle;

  public LowerArmIntake(IntakeSubsystem intakeSubsystem, double angle) {
    this.intakeSubsystem = intakeSubsystem;
    this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setArmAngle(angle);
    //CommandScheduler.getInstance().schedule(new StartIntake(intakeSubsystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setArmAngle(angle);
    System.out.println("hi"); 
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    armPositionAngle = intakeSubsystem.encoderGetAngle();
    return Math.abs(armPositionAngle - angle) < Constants.IntakeConstants.TOLERANCE;
  }
}
