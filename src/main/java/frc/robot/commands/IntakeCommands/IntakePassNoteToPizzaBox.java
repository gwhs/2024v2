// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePassNoteToPizzaBox extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;
  private PizzaBoxSubsystem pizzaBoxSubsystem;
  private boolean prevSensorValue = false;
  private boolean currentSensorValue = false;
  private boolean noteLatch = false;
  private double timer = 0;

  public IntakePassNoteToPizzaBox(IntakeSubsystem subsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    intakeSubsystem = subsystem;
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pizzaBoxSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.spinIntakeMotor(.7, 100);
    timer = Timer.getFPGATimestamp(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
    pizzaBoxSubsystem.stopPizzaBoxMotor();
    intakeSubsystem.stopIntakeMotors();
    if(interrupted) {
      new IntakeRejectNote(intakeSubsystem).schedule();
    }
  }

  // Returns true when the command should end; called every cycle
  @Override
  public boolean isFinished() {

    // return false;
    prevSensorValue = currentSensorValue;
    currentSensorValue = intakeSubsystem.isNotePresent();

    if(prevSensorValue == true && currentSensorValue == false) {
      noteLatch = true;
    }  
    if( timer + 5 < Timer.getFPGATimestamp() && noteLatch) {
      noteLatch = false;
      return true; 
    }
    return false;
  }

}
