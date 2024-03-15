// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SpinArmAndPizzaBox extends Command {
  private PizzaBoxSubsystem pizzaBoxSubsystem;
  private ArmSubsystem armSubsystem;
  private double angle;
  private double vel;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //velocity = 100 for testing shooting 
  public SpinArmAndPizzaBox(PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, double angle, double vel) {
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;
    this.armSubsystem = armSubsystem;
    this.angle = angle;
    this.vel = vel;
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.targetArmAngle(angle);
    pizzaBoxSubsystem.spinPizzaBoxMotor(-5, 100);
  }

  @Override
  public void execute() {
    if(armSubsystem.encoderGetAngle() > PizzaBoxSubsystem.PizzaBox.START_SPIN_DEGREE)
    {
      pizzaBoxSubsystem.spinPizzaBoxMotor(vel, 500);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getController().atGoal();
  } 
}
