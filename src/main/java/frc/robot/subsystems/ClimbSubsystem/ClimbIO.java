package frc.robot.subsystems.ClimbSubsystem;

public interface ClimbIO {

  Object m_leftClimbMotor = null;

public double getRightMotorPosition();

  public double getLeftMotorPosition();

  public void setLeftMotorSpeed(double speed);

  public void setRightMotorSpeed(double speed);

  public void update();

  public void setPositionLeft(double position);

  public void setPositionRight(double position);

}