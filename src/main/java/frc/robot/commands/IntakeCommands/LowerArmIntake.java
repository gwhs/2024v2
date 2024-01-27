// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class LowerArmIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSubsystem;

  private double motorAng;
  private double angle;
  private double tolerance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public LowerArmIntake(IntakeSubsystem intakeSubsystem, double angle, double tolerance) {
    this.intakeSubsystem = intakeSubsystem;
    this.angle = angle;
    this.tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new StartIntake(intakeSubsystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  //setting the angle of the arm motor
  @Override
  public void execute() {
    intakeSubsystem.setArmAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    motorAng = intakeSubsystem.getArmPos();
    return Math.abs(motorAng - angle) < tolerance;
  }
}
