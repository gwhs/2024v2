// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

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
    System.out.println("hello");
    armSubsystem.targetArmAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(armSubsystem.getController().atGoal())
    // {

    // }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return armSubsystem.getController().atGoal(); 
  }
}
