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
    timer = Timer.getFPGATimestamp(); 
    addRequirements(pizzaBoxSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pizzaBoxSubsystem.spinPizzaBoxMotor(-50, 10);
    intakeSubsystem.spinIntakeMotor(.7, 100);
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
    pizzaBoxSubsystem.stopPizzaBoxMotor();
    intakeSubsystem.stopIntakeMotors();
    if(interrupted) {
      intakeSubsystem.rejectIntake(1, 0);
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
    if( timer + 3 < Timer.getFPGATimestamp() && noteLatch) {
      noteLatch = false;
      return true; 
    }
    return false;
  }

}
