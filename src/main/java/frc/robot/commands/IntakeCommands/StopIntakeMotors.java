// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StopIntakeMotors extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;

  public StopIntakeMotors(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  // sets the intake arm to 0 degrees
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // spins to intake the note
  @Override
  public void execute() {
    intakeSubsystem.stopIntakeMotors();
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end; called every cycle
  // stop spinning once note is decected by sensor
  @Override
  public boolean isFinished() {
    return true;
  }

}
