// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimeLight;

import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class parallelTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem driSwerveSubsystem;
  private final ApriltagController apriltagController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public parallelTag(SwerveSubsystem driSwerveSubsystem, ApriltagController apriltagController) {
    this.driSwerveSubsystem = driSwerveSubsystem;
    this.apriltagController = apriltagController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driSwerveSubsystem, apriltagController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltagController.setTolerance("rotation");
    apriltagController.setPoint(0, "rotation");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = apriltagController.updatePIDRotationTest();
    driSwerveSubsystem.drive(new Translation2d(0, 0), angle, false);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return apriltagController.atSetpoint("rotation");
  }
}