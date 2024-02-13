package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingForward extends Command{

  private double motorAng;
  private double angle;
  private double velocity;
  private double acceleration;
  private double tolerance;

  

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
    // Called when the command is initially scheduled.
    

  public SwingForward(ArmSubsystem armSubsystem, double angle, double velocity, double acceleration, double tolerance)
  {
    this.armSubsystem = armSubsystem;
    this.angle = angle;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.tolerance = tolerance;
    addRequirements(armSubsystem);
  }
  public void initialize() {
    armSubsystem.setAngle(angle, velocity, acceleration);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    motorAng = armSubsystem.encoderGetAngle();
    // System.out.println("Check Finished " + "motorAngle = " + motorAng + " EncoderAngle = " + angle);
    // System.out.println("delta = " + (motorAng - angle));
    return Math.abs(motorAng - angle) < tolerance;
  }
}