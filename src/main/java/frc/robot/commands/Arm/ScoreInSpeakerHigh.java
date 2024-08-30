// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreInSpeakerHigh extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public ScoreInSpeakerHigh(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
      new ScoreInSpeakerAdjustable(pizzaBoxSubsystem, armSubsystem, ArmSubsystem.Arm.SPEAKER_HIGH_ANGLE)
    );
  }

  
}
