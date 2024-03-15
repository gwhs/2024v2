// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreInSpeakerAdjustable extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public ScoreInSpeakerAdjustable(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, double angle) {
    addCommands(
      new SpinArmAndPizzaBox(pizzaBoxSubsystem, armSubsystem, angle, 100).withTimeout(2),
      new SpinNoteContainerMotor(pizzaBoxSubsystem, 100, 100),
      Commands.waitUntil(()->pizzaBoxSubsystem.isAtVelocity(90)).withTimeout(0.5),
      new SwingForwardServo(pizzaBoxSubsystem),
      Commands.waitSeconds(.2),
      new SwingBackServo(pizzaBoxSubsystem),
      Commands.waitSeconds(0.2),
      new StopNoteContainerMotor(pizzaBoxSubsystem),
      new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.INTAKE_ANGLE).withTimeout(0.1),
      Commands.runOnce(() -> {
        pizzaBoxSubsystem.hasNote = false;
        })
    );
  }  
}
