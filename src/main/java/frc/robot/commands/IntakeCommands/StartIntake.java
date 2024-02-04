// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class StartIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem IntakeSubsystem;
  private boolean prevSensorValue;
  private boolean currentSensorValue;
  private boolean noteLatch;
  private int counter;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartIntake(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.spinIntakeMotor();
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stopIntakeMotors();
    //CommandScheduler.getInstance().schedule(new UpperArmIntake(IntakeSubsystem));
  }

  // Returns true when the command should end.
  // called every cycle
  @Override
  public boolean isFinished() {
    prevSensorValue = currentSensorValue;
    currentSensorValue = IntakeSubsystem.getSensor();

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
