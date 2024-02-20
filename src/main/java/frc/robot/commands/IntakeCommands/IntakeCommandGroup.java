// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommandGroup extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public IntakeCommandGroup(IntakeSubsystem intakeSubsystem) {
    
    final PIDController intakeController = new PIDController(.01, .001, .0);
    intakeController.setTolerance(Constants.IntakeConstants.TOLERANCE);
    
    addCommands(

    // spin intake and lower intake to ground will happen at the same time
      new ParallelCommandGroup(
        new SpinIntakePID(intakeController, intakeSubsystem, 0), 
        new IntakePickUpFromGround(intakeSubsystem).withTimeout(2)
      ),

      new SpinIntakePID(intakeController, intakeSubsystem, Constants.IntakeConstants.MAX_ARM_ANGLE),
      new IntakePassNoteToPizzaBox(intakeSubsystem)
    );
  
  }
}
