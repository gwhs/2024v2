// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StartStop extends SequentialCommandGroup {
  
  /** Creates a new StartStop. */
  public StartStop( int cycleAmount, double secondsInbetween  , IntakeSubsystem intakeSubsystem) {
    for( int i = 0 ; i < cycleAmount ; i ++ ){
      addCommands( 
        new IntakeNote(intakeSubsystem), 
        Commands.waitSeconds(secondsInbetween),
        new StopIntakeMotors(intakeSubsystem),
        Commands.waitSeconds(secondsInbetween)); 
    }

  }

}
