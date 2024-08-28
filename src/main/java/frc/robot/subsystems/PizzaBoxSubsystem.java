package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.UtilMotor;

public class PizzaBoxSubsystem extends SubsystemBase {


    public static final int PIZZABOX_ID = 23;
    public static final int SERVO_PWN_SLOT = 0;
    public static final int SERVO2_PWN_SLOT = 1;
    public static final int START_SPIN_DEGREE = 180;
    public boolean hasNote = false;
    private TalonFX PB;
    private Servo PBServo;
    private Servo PBServo2;
  

  public PizzaBoxSubsystem(int pizzaBoxId, String pizzaBoxCanbus, int channelServo, int channelServoTheSecond)
  {
    PB = new TalonFX(pizzaBoxId, pizzaBoxCanbus);
    PBServo2 = new Servo(channelServoTheSecond);
    PBServo = new Servo(channelServo);
    UtilMotor.configMotor(PB, .5, 0, 0, .12, 15, 80, false);      
   
  }

  public void SpinPBMotor(double velocity, double acceleration) {
    PB.set(velocity/100);
  }

  public void SetPBServo(double angle) {
    PBServo.setAngle(angle);
  }
   public void SetPBServo2(double angle) {
    PBServo2.setAngle(angle);
  }

  public double PBServoAngle() {
    return PBServo.getAngle();
  }

  public double PBServo2Angle() {
    return PBServo2.getAngle();
  }

  public void StopPBMotor() {
    PB.stopMotor();
  }

  public boolean AtVelocity(double vel) {
    return PB.getRotorVelocity().getValue() >= vel;
  }

  @Override
  public void periodic() {  }
}