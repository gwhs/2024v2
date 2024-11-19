package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
    
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);

    motorOutput.NeutralMode = NeutralModeValue.Coast;

    m_armMotor.getConfigurator().apply(currentConfig);
    m_armMotor.getConfigurator().apply(currentConfig);

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

  public boolean isEncoderConnected() {
    return m_armEncoder.isConnected();
  }

  public boolean isOutOfBounds() {
    if (isEncoderConnected() == false) {
      return false;
    }
    if (getArmEncoderAngle() >= ArmConstants.ARM_ANGLE_MAX) {
      return true;
    }
    if (getArmEncoderAngle() <= ArmConstants.ARM_ANGLE_MIN) {
      return true;
    }
    // 1) encoder disconnnected - return false
    // 2) encoder is above max angle - return true
    // 3) encoder is below min angle - return true
    // 4) encoder in within max and min - return false
    return false;
  }

}
