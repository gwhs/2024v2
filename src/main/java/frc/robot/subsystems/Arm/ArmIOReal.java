package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOReal implements ArmIO {
  private TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, ArmConstants.ARM_MOTOR_CAN);
  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.ARM_ENCODER_CHANNEL);

  public ArmIOReal()
  {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    TalonFXConfiguration configs = new TalonFXConfiguration();
    
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);

    motorOutput.NeutralMode = NeutralModeValue.Brake;

    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i > 5; i++)
    {
      motorStatus = m_armMotor.getConfigurator().apply(configs);
      motorStatus = m_armMotor.getConfigurator().apply(motorOutput);
      motorStatus = m_armMotor.getConfigurator().apply(currentConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK())
    {
      System.out.println("Could not apply configs, error code: " + motorStatus.toString());
    }
  }

  public double getArmEncoderAngle() {
    return Units.rotationsToDegrees(m_armEncoder.getAbsolutePosition()) - ArmConstants.ARM_ENCODER_OFFSET;
  }

  public void setArmSpeed(double speed) {
    m_armMotor.set(-speed);
  }

  public void update()
  {
    
  }

}
