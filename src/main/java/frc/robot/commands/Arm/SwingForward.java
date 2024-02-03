package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingForward extends Command{

  //setArmPosition(startAngle, goalAngle), velocity
/*
 * Goal - Reset the arm back to neutral position to intake from ground
 * What we ant to do: Set arm angle to be back at 0. NOT just encoder angle. 
 */
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

  //NOTE : potentially needs to swing 270 degrees 
  @Override
  public void execute() {
    System.out.println("Swing Forward successful ");
    //System.out.println("Encoder pos: ");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    motorAng = armSubsystem.encoderGetAngle();
    return Math.abs(motorAng - angle) < tolerance;
  }
}