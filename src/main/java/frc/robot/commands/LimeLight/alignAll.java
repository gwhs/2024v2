// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimeVision.ApriltagController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class alignAll extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public alignAll(SwerveSubsystem driSwerveSubsystem, ApriltagController apriltagController) {
    addCommands(
        driSwerveSubsystem.driveToPose(apriltagController.getTargetPose()),
        Commands.waitSeconds(0.1),
        new align(driSwerveSubsystem, apriltagController)
    );
  }
}
