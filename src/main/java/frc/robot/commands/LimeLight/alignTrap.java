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
public class alignTrap extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem driSwerveSubsystem;
  private final ApriltagController apriltagController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public alignTrap(SwerveSubsystem driSwerveSubsystem, ApriltagController apriltagController) {
    this.driSwerveSubsystem = driSwerveSubsystem;
    this.apriltagController = apriltagController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driSwerveSubsystem, apriltagController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltagController.setTolerance("forward");
    apriltagController.setPoint(ApriltagConstants.TargetDistance.FORWARD_TARGET, "forward"); // fixed x distance from tag before crashing field perimeter
    apriltagController.setTolerance("sideways");
    apriltagController.setPoint(ApriltagConstants.TargetDistance.SIDEWAYS_TARGET, "sideways");
    apriltagController.setTolerance("rotation");
    apriltagController.setPoint(ApriltagConstants.TargetDistance.ROTATION_TARGET, "rotation");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = apriltagController.updatePIDForward();
    double sideways = apriltagController.updatePIDSideways();
    double angle = apriltagController.updatePIDRotationTest();

    driSwerveSubsystem.drive(new Translation2d(-forward, sideways), angle, false);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return apriltagController.atSetpoint("forward") && apriltagController.atSetpoint("rotation");
  }
}