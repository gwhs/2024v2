// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class IntakeEmergencyStop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeSubsystem intakeSubsystem;

  public IntakeEmergencyStop(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Command i = CommandScheduler.getInstance().requiring(intakeSubsystem);
    if(i != null)
    {
      i.cancel();
    }

    intakeSubsystem.stopArmMotor();
    intakeSubsystem.emergencyStop = !intakeSubsystem.emergencyStop;

    if(intakeSubsystem.emergencyStop == false) {
      new IntakeResetArm(intakeSubsystem).schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  // runs once when isFinished is called
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // called every cycle
  @Override
  public boolean isFinished() {
    return true;
  }

}