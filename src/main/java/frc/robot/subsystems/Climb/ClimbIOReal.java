package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbIOReal implements ClimbIO {
  private final TalonFX m_leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
  private final TalonFX m_rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
  private final DigitalInput bottomLeft = new DigitalInput(ClimbConstants.BOT_LEFT_LIMIT_ID);
  private final DigitalInput bottomRight = new DigitalInput(ClimbConstants.BOT_RIGHT_LIMIT_ID);
  private final DigitalInput topLeft = new DigitalInput(ClimbConstants.TOP_LEFT_LIMIT_ID);
  private final DigitalInput topRight = new DigitalInput(ClimbConstants.TOP_RIGHT_LIMIT_ID);
  
  
  public double getRightMotorPosition() {
    return m_rightClimbMotor.getPosition().getValueAsDouble();
  }

  public double getLeftMotorPosition() {
    return m_leftClimbMotor.getPosition().getValueAsDouble();
  }

  public void setLeftMotorSpeed(double speed) {
    m_leftClimbMotor.set(speed);
  }

  public void setRightMotorSpeed(double speed) {
    m_rightClimbMotor.set(speed);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean getTopLeftLimitSwitch() {
    return getTopLeftLimitSwitch();
  }

  @Override
  public boolean getTopRightLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTopRightLimitSwitch'");
  }

  @Override
  public boolean getBottomLeftLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBottomLeftLimitSwitch'");
  }

  @Override
  public boolean getBottomRightLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBottomRightLimitSwitch'");
  }
}