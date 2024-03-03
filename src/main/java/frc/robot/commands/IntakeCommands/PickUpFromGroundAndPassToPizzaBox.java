// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.Constants;
import frc.robot.commands.Arm.*;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PickUpFromGroundAndPassToPizzaBox extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public PickUpFromGroundAndPassToPizzaBox(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    addCommands(
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.INTAKE_ANGLE).withTimeout(3)
        .alongWith(new IntakePickUpFromGroundPID(intakeSubsystem, .7, 0.0))
        .alongWith(new SpinNoteContainerMotor(pizzaBoxSubsystem, -70, 100))
        .andThen(new StopIntakeMotors(intakeSubsystem))
        .andThen(new SpinIntakePID(intakeSubsystem, Constants.IntakeConstants.UP_POSITION))
        .andThen(new IntakePassNoteToPizzaBox(intakeSubsystem, pizzaBoxSubsystem).withTimeout(3))
        .andThen(new StopNoteContainerMotor(pizzaBoxSubsystem))
        .andThen(new StopIntakeMotors(intakeSubsystem))
        );
  }
//.andThen((new IntakePassNoteToPizzaBox(intakeSubsystem, pizzaBoxSubsystem)).withTimeout(5)
  
}
