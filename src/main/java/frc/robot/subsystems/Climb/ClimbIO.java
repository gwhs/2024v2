package frc.robot.subsystems.Climb;

public interface ClimbIO {

  public double getRightMotorPosition();

  public double getLeftMotorPosition();

  public void setLeftMotorSpeed(double speed);

  public void setRightMotorSpeed(double speed);

  public void update();

  public boolean getTopLeftLimitSwitch(); 

  public boolean getTopRightLimitSwitch(); 
  
  public boolean getBottomLeftLimitSwitch(); 

  public boolean getBottomRightLimitSwitch(); 


}