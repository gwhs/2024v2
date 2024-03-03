// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.subsystems.LimeVision.ApriltagConstants;
import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class align extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveSubsystem driSwerveSubsystem;
  private final ApriltagController apriltagController;

  public align(SwerveSubsystem driSwerveSubsystem, ApriltagController apriltagController) {
    this.driSwerveSubsystem = driSwerveSubsystem;
    this.apriltagController = apriltagController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driSwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltagController.setTolerance("forward");
    apriltagController.setPoint("forward");
    apriltagController.setTolerance("sideways");
    apriltagController.setPoint("sideways");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driSwerveSubsystem.drive(apriltagController.fieldRelative(apriltagController.updatePIDForward(), apriltagController.updatePIDSideways()), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // this or when only forward is finished
    return apriltagController.atSetpoint("forward") && apriltagController.atSetpoint("sideways");
  }
}
