// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SpinToArmAngle extends Command {

  private double angle;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  public SpinToArmAngle(ArmSubsystem armSubsystem, double angle) {
    this.armSubsystem = armSubsystem;
    this.angle = angle;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.targetArmAngle(angle);
  }

  @Override
  public void end(boolean interrupted){
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getController().atGoal(); 
  }
}
