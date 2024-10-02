package frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOReal implements ClimbIO {
  private TalonFX m_leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
  private TalonFX m_rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");

  public ClimbIOReal() {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    
    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator leftClimbConfigurator = m_leftClimbMotor.getConfigurator();
    leftClimbConfigurator.apply(motorOutput);
    leftClimbConfigurator.apply(currentConfig);


    TalonFXConfigurator rightClimbConfigurator = m_rightClimbMotor.getConfigurator();
    rightClimbConfigurator.apply(motorOutput);
    rightClimbConfigurator.apply(currentConfig);


    

  }

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
}