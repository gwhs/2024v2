// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreInAmp extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public ScoreInAmp(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.AMP_ANGLE),
        new SpinNoteContainerMotor(pizzaBoxSubsystem, 26, 50),
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.INTAKE_ANGLE)
    );
  }

  
}
