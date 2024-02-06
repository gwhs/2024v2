package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class SwingServo extends Command{

  //setArmPosition(startAngle, goalAngle), velocity
/*
 * Goal - Reset the arm back to neutral position to intake from ground
 * What we ant to do: Set arm angle to be back at 0. NOT just encoder angle. 
 */
  private double motorAng;
  private double angle;
  private double speed;
  private double tolerance;

  

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
    // Called when the command is initially scheduled.
    

  public SwingServo(ArmSubsystem armSubsystem)
  {
    this.armSubsystem = armSubsystem;
    //this.tolerance = tolerance;
    addRequirements(armSubsystem);
  }
  public void initialize() {
    if(armSubsystem.getServoAngle() < 180) {
      armSubsystem.setServoAngle(180);
    } else {
      armSubsystem.setServoAngle(0);
    }
    System.out.println(armSubsystem.getServoSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.

  //NOTE : potentially needs to swing 270 degrees 
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
    motorAng = armSubsystem.getServoAngle();
    return motorAng == 0.0  || motorAng == 180.0;
  }
} 