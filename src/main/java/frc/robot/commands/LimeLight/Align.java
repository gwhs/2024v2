package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AprilTagConstants;

import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.commands.driveCommands.rotateinPlace;
import frc.robot.commands.LimeLight.FaceAprilTag;
import frc.robot.commands.LimeLight.Sideways;


public class Align extends SequentialCommandGroup {
    
    public Align(
        SwerveSubsystem driSwerveSubsystem, 
        LimeLightSub limeLightSub) {
            // Add commands
        addCommands(
            new rotateinPlace(() -> AprilTagConstants.APRILTAG_ROTATION[limeLightSub.getID()], driSwerveSubsystem),
            new Sideways(driSwerveSubsystem, limeLightSub),
            new FaceAprilTag(driSwerveSubsystem, limeLightSub));
    }
}
