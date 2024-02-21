package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AprilTagConstants;

import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.commands.driveCommands.rotateinPlace;
import frc.robot.commands.LimeLight.FaceAprilTag;
import frc.robot.commands.LimeLight.Sideways;
import frc.robot.commands.LimeLight.Forward;


public class Align extends SequentialCommandGroup {
    
    public Align(
        SwerveSubsystem driSwerveSubsystem, 
        ApriltagController apriltagController) {
            // Add commands
        addCommands(
            new rotateinPlace(() -> apriltagController.getApriltagHeading(), driSwerveSubsystem),
            Commands.waitSeconds(0.5),
            new Sideways(driSwerveSubsystem, apriltagController).andThen(new Forward(driSwerveSubsystem, apriltagController)));
    }
}
