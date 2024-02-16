// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.StopNoteContainerMotor;
import frc.robot.commands.Arm.SwingBack;
import frc.robot.commands.Arm.SwingForward;
import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SpinAndSwing extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public SpinAndSwing(ArmSubsystem armSubsystem) {
    addCommands(
        new SpinNoteContainerMotor (armSubsystem, 100, 150),
        Commands.waitSeconds(.5), 
        new SwingForwardServo(armSubsystem),
        new SwingBackServo(armSubsystem),
        new StopNoteContainerMotor(armSubsystem)
        );

  }

  
}