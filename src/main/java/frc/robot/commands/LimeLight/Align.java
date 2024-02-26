package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
            new Sideways(driSwerveSubsystem, apriltagController),
            Commands.waitSeconds(0.5),
            new wheelAlignForward(driSwerveSubsystem, apriltagController).withTimeout(0.1),
            new Forward(driSwerveSubsystem, apriltagController));
            // new Sideways(driSwerveSubsystem, apriltagController).andThen(new ParallelCommandGroup(new ForwardTA(driSwerveSubsystem, apriltagController), new FaceAprilTag(driSwerveSubsystem, apriltagController))));
            // new Sideways(driSwerveSubsystem, apriltagController).andThen(new ParallelCommandGroup(new Forward(driSwerveSubsystem, apriltagController), new FaceAprilTag(driSwerveSubsystem, apriltagController))));
    }
}