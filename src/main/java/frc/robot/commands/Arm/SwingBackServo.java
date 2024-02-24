package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingBackServo extends Command{
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
    // Called when the command is initially scheduled.
    

  public SwingBackServo(ArmSubsystem armSubsystem)
  {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }
  public void initialize() {   
    armSubsystem.setServoAngle(50);
  }
  

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double motorAng = armSubsystem.getServoAngle();
  
    return Math.abs(motorAng-50) < .001;
  } 
}