/*
 * Does not use robot pose, dependent on driver getting in range of tag
 */

package frc.robot.commands.LimeLight;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.commands.driveCommands.rotateinPlace;
import frc.robot.commands.LimeLight.Sideways;
import frc.robot.commands.LimeLight.Forward;


public class alignTrap extends ParallelDeadlineGroup {
    
    public alignTrap(
        SwerveSubsystem driSwerveSubsystem, 
        ApriltagController apriltagController) {
            // Add commands
        super(new Forward(driSwerveSubsystem, apriltagController));
        
        addCommands(
            new Sideways(driSwerveSubsystem, apriltagController),
            new parallelTag(driSwerveSubsystem, apriltagController),
            new Forward(driSwerveSubsystem, apriltagController));
    }
}