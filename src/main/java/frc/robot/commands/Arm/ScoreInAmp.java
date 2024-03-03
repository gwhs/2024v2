// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreInAmp extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public ScoreInAmp(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.AMP_ANGLE).withTimeout(3),
        new SpinNoteContainerMotor(pizzaBoxSubsystem, 40, 100),
        Commands.waitSeconds(.5),
        new SwingForwardServo(pizzaBoxSubsystem),
        Commands.waitSeconds(.2),
        new SwingBackServo(pizzaBoxSubsystem),
        new StopNoteContainerMotor(pizzaBoxSubsystem),
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.INTAKE_ANGLE)
    );

    pizzaBoxSubsystem.hasNote = false;
  }

  
}
