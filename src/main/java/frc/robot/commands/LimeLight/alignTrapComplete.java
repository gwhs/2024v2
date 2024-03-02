// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;
import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class alignTrapComplete extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public alignTrapComplete(SwerveSubsystem driSwerveSubsystem, ApriltagController apriltagController) {
    addCommands(
        new parallelTag(driSwerveSubsystem, apriltagController),
        new alignTrap(driSwerveSubsystem, apriltagController)
    );
  }
  
}
