package frc.robot.subsystems.Arm;

public interface ArmIO {
    public double getArmEncoderAngle();
    public void setArmSpeed(double speed);
    public void update();
}
