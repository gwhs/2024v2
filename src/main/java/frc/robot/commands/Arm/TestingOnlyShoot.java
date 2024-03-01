// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.StopNoteContainerMotor;
import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestingOnlyShoot extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public TestingOnlyShoot(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, double vel) {
    addCommands(
        new SpinNoteContainerMotor(pizzaBoxSubsystem, vel, 500),
        Commands.waitSeconds(0.7),
        new SwingForwardServo(pizzaBoxSubsystem),
        Commands.waitSeconds(.2),
        new SwingBackServo(pizzaBoxSubsystem),
        new StopNoteContainerMotor(pizzaBoxSubsystem),
        new SpinToArmAngle(armSubsystem, ArmSubsystem.Arm.INTAKE_ANGLE)
        );
  }

  
}
