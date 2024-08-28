package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class ResetArm extends Command{
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  private final PizzaBoxSubsystem pizzaBoxSubsystem;

  public ResetArm(ArmSubsystem armSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem)
  {
    this.armSubsystem = armSubsystem;
    this.pizzaBoxSubsystem = pizzaBoxSubsystem;

  }
  
  public void initialize() {
    DataLogManager.log("Command Start: Reset Arm");
    Command c = CommandScheduler.getInstance().requiring(armSubsystem);
    if(c != null) {
      c.cancel();
    }

    Command p = CommandScheduler.getInstance().requiring(pizzaBoxSubsystem);
    if(p != null) {
      p.cancel();
    }
    
    armSubsystem.stopArmMotor();
    pizzaBoxSubsystem.StopPBMotor();
    armSubsystem.targetArmAngle(ArmSubsystem.Arm.INTAKE_ANGLE);
    pizzaBoxSubsystem.SetPBServo(50);
    pizzaBoxSubsystem.SetPBServo2(0);
  }
  

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("Command End: Reset Arm");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  } 
}