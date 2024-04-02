// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SpinArmCommand extends Command {
  /** Creates a new SpinArmCommand. */
  private final ArmSubsystem m_ArmSubsystem;
  public SpinArmCommand(ArmSubsystem m_ArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = m_ArmSubsystem;
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.booster = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.booster = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
