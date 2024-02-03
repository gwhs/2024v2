// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.LimeLight.DriveToTag;
import frc.robot.commands.LimeLight.FaceAprilTag;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveThere extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  public DriveThere(
      SwerveSubsystem swerve,
      LimeLightSub limeLightSub)
      {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        new DriveToTag(swerve, limeLightSub, () -> false), 
        new FaceAprilTag(swerve, limeLightSub, () -> false));

    //
  }
}