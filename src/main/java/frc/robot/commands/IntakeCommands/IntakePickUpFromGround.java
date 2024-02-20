// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* 
 * should bring the intake down to 0 degrees, and start spinning the intake motors
 * will stop once note in dectected in intake
 */

public class IntakePickUpFromGround extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;
  private boolean sensorValue;

  public IntakePickUpFromGround(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  // sets the intake arm to 0 degrees
  @Override
  public void initialize() {
    if(intakeSubsystem.encoderGetAngle() > 0 || intakeSubsystem.encoderGetAngle() < 0) {
      new SpinIntakePID(intakeSubsystem, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // spins to intake the note
  @Override
  public void execute() {
    intakeSubsystem.spinIntakeMotor(100, 5);
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeMotors();
  }

  // Returns true when the command should end; called every cycle
  // stop spinning once note is decected by sensor
  @Override
  public boolean isFinished() {
    sensorValue = intakeSubsystem.isNotePresent();
    if(sensorValue) {
      System.out.println("sensor");
    }
    return sensorValue;
  }

}
