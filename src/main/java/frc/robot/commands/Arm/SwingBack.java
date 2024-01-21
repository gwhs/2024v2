package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingBack extends Command{

  //setArmPosition(startAngle, goalAngle), velocity

  private double motorAng;
  private double angle;
  private double velocity;
  private double acceleration;
  private double tolerance;

  

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
    // Called when the command is initially scheduled.
    
//  OLD CONSTRUCTOR HEADING : public SwingBack(ArmSubsystem armSubsystem) {
  public SwingBack(ArmSubsystem armSubsystem, double angle, double velocity, double acceleration, double tolerance)
  {
    this.armSubsystem = armSubsystem;
    this.angle = angle;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.tolerance = tolerance;
    addRequirements(armSubsystem);
  }
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.

  //NOTE : potentially needs to swing 270 degrees 
  @Override
  public void execute() {
    //armSubsystem.setAng(angle, velocity, acceleration);
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