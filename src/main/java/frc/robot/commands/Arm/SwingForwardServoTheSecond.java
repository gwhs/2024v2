package frc.robot.commands.Arm;
import frc.robot.subsystems.PizzaBoxSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingForwardServoTheSecond extends Command{
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PizzaBoxSubsystem pizzaBoxSubsystem;

  public SwingForwardServoTheSecond(PizzaBoxSubsystem pizzaBoxSubsystem)
  {
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;
    addRequirements(pizzaBoxSubsystem);
  }
  public void initialize() {
    pizzaBoxSubsystem.setServoAngleTheSecond(180);
  }
  
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    double motorAng = pizzaBoxSubsystem.getServoAngle();
    return Math.abs(motorAng - 180) < .001;
  }
}