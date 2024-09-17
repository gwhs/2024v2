package frc.robot.subsystems.ClimbSubsystem;

public interface ClimbIO {

  public double getRightMotorPosition();

  public double getLeftMotorPosition();

  public void setLeftMotorSpeed(double speed);

  public void setRightMotorSpeed(double speed);

}