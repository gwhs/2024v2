package frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOReal implements ClimbIO{
    private TalonFX m_leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private TalonFX m_rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");

    public double getRightMotorPosition(){
        return m_rightClimbMotor.getPosition().getValueAsDouble();
    }

    public double getLeftMotorPosition(){
        return m_leftClimbMotor.getPosition().getValueAsDouble();
    }

    public void setLeftMotorSpeed(double speed) {
        m_leftClimbMotor.set(speed);
    }
    public void setRightMotorSpeed(double speed) {
        m_rightClimbMotor.set(speed);
    }
}