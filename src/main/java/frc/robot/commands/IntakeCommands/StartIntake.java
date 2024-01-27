// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class StartIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem IntakeSubsystem;
  private double velocity;
  private double acceleration;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartIntake(IntakeSubsystem subsystem, double vel, double acc) {
    IntakeSubsystem = subsystem;
    velocity = vel;
    acceleration = acc;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.spinIntakeMotor(velocity, acceleration);
  }

  // Called once the command ends or is interrupted.
  //using limit switches
  @Override
  public void end(boolean interrupted) {
    // top limit is tripped so stop
    IntakeSubsystem.stopIntakeMotors();
  }

  public void tempStop(){
    if(!IntakeSubsystem.getSensor()) {
      IntakeSubsystem.stopIntakeMotors();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IntakeSubsystem.getSensor(); 
  }

  //button to stop isFinished command
}
