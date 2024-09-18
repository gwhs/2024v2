package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOReal implements ArmIO {
  private TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, ArmConstants.ARM_MOTOR_CAN);
  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.ARM_ENCODER_CHANNEL);

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