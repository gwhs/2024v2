// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreInTrapStutter extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public ScoreInTrapStutter(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.TRAP_ANGLE).withTimeout(3));


    for(int index = 0; index < 8; index++)
    {
      addCommands(
        new SpinNoteContainerMotor(pizzaBoxSubsystem, 26, 100),
        Commands.waitSeconds(.2),
        new StopNoteContainerMotor(pizzaBoxSubsystem),
        Commands.waitSeconds(.2)
      );
    }

    addCommands(
        new StopNoteContainerMotor(pizzaBoxSubsystem),
        new SpinToArmAngle(armSubsystem, 135)
    );
  }  
}
