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
import frc.robot.commands.LimeLight.faceTag;


public class Align extends SequentialCommandGroup {
    
    public Align(
        SwerveSubsystem driSwerveSubsystem, 
        ApriltagController apriltagController) {
            // Add commands
        addCommands(
            // new rotateinPlace(() -> AprilTagConstants.APRILTAG_ROTATION[apriltagController.getID()], driSwerveSubsystem),
            // Commands.waitSeconds(0.5),
            // new faceTag(driSwerveSubsystem, apriltagController).alongWith(new Sideways(driSwerveSubsystem, apriltagController)),
            // Commands.waitSeconds(0.5),
            new Sideways(driSwerveSubsystem, apriltagController),
            Commands.waitSeconds(0.5),
            Commands.print("moving"),
            new FaceAprilTag(driSwerveSubsystem, apriltagController).andThen(new Forward(driSwerveSubsystem, apriltagController)),
            Commands.waitSeconds(0.5),
            Commands.print("facetag"),
            new FaceAprilTag(driSwerveSubsystem, apriltagController));
    }
}
