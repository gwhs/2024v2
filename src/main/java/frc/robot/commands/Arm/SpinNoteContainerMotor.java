// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinNoteContainerMotor extends Command {

  private double velocity;
  private double acceleration;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  public SpinNoteContainerMotor(ArmSubsystem armSubsystem, double velocity, double acceleration) {
    this.armSubsystem = armSubsystem;
    this.velocity = velocity;
    this.acceleration = acceleration;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.spinPizzaBoxMotor(velocity, acceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Sensor is inactive?
    return true;
  }
}
