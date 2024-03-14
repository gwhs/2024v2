// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StartStop extends SequentialCommandGroup {
  private int cycleAmount ;
  private double secondsInbetween ; 
  private final IntakeSubsystem  intakeSubsystem ; 
  
  /** Creates a new StartStop. */
  public StartStop( int cycleAmount, double secondsInbetween  , IntakeSubsystem intakeSubsystem) {
    super(new IntakeNote(intakeSubsystem));

    this.cycleAmount = cycleAmount ; 
    this.secondsInbetween = secondsInbetween ; 
    this.intakeSubsystem = intakeSubsystem ; 

    for( int i = 0 ; i < cycleAmount ; i ++ ){
      addCommands( new IntakeNote(intakeSubsystem), Commands.waitSeconds((int)secondsInbetween)); 
    }

  }

}
