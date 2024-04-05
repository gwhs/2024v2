package frc.robot.commands.Arm;

import frc.robot.subsystems.PizzaBoxSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SwingBackServoTheSecond extends Command{
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PizzaBoxSubsystem pizzaBoxSubsystem;

  public SwingBackServoTheSecond(PizzaBoxSubsystem pizzaBoxSubsystem)
  {
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;
    addRequirements(pizzaBoxSubsystem);
  }

  public void initialize() {   
    pizzaBoxSubsystem.setServoAngleTheSecond(0);
  }
  
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  } 
}