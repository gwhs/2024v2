// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class IntakePassNoteToPizzaBox extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private boolean prevSensorValue;
  private boolean currentSensorValue;
  private boolean noteLatch;
  private int counter; 

  public IntakePassNoteToPizzaBox(IntakeSubsystem subsystem, ArmSubsystem armSubsystem) {
    intakeSubsystem = subsystem;
    this.armSubsystem = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinIntakeMotor(20, 5);
    armSubsystem.spinPizzaBoxMotor(-1, 5);
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeMotors();
  }

  // Returns true when the command should end; called every cycle
  @Override
  public boolean isFinished() {
    prevSensorValue = currentSensorValue;
    currentSensorValue = intakeSubsystem.isNotePresent();

    if(prevSensorValue == true && currentSensorValue == false) {
      noteLatch = true;
    }  
    // two second delay before checking sensor again
    if(counter > Constants.IntakeConstants.NOTE_DELAY && noteLatch) {
      noteLatch = false;
      return true; 
    }
    else {
      counter++;
    }
    return false;
  }

}
