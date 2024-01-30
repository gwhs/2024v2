package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingBack extends Command{

  //setArmPosition(startAngle, goalAngle), velocity
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  private double velocity;
  private double acceleration;
  

    // Called when the command is initially scheduled.
    
  public SwingBack( ArmSubsystem armSubsystem, double velocity, 
  double acceleration) {
    this.armSubsystem = armSubsystem;
    this.velocity = velocity;
    this.acceleration = acceleration;
    addRequirements(armSubsystem);
  }
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setAngle(0, velocity, acceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
