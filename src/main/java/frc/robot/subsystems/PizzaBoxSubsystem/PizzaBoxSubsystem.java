package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;

// COMMANDS NEED TO BE RUN WITH LAMBDAS
public class PizzaBoxSubsystem extends SubsystemBase {
  
  private TalonFX m_PizzaBoxMotor;
  private Servo PBservo;
  private Servo PBFlapServo;

  public PizzaBoxSubsystem() {
    m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PIZZA_BOX_ID,PizzaBoxConstants.PIZZA_BOX_CAN);
    PBservo = new Servo(PizzaBoxConstants.SERVO_PWD);
    PBFlapServo = new Servo(PizzaBoxConstants.FLAP_PWD);

  }

  public Command reset_command() {
    return this.runOnce(() -> reset(PBservo,PBFlapServo,m_PizzaBoxMotor));
  }

  public Command spit_command(TalonFX motor) {
    return this.runOnce(() -> motor.set(1));
  }

  public Command slurp_command(TalonFX motor) {
    return this.runOnce(() -> motor.set(-1));
  }

  public Command stopMotor(TalonFX motor) {
    return this.run(() -> motor.set(.00));
  }

  public static double motorSpeed(TalonFX motor) {
    return motor.getRotorVelocity().getValue();
  }

  public Command stopFlap(Servo servo) {
    return this.run(() -> servo.set(PizzaBoxConstants.STOP));
  }
public Command stopKicker(Servo servo) {
    return this.run(() -> servo.set(PizzaBoxConstants.STOP));
  }

  public Command speedyArm_Command(TalonFX m) {
    return this.run(() -> SpeedyArm(m));

  }

  public static String reset(Servo s, Servo s2, TalonFX f) {
    s.set(PizzaBoxConstants.SERVO_NORM);
    s2.set(PizzaBoxConstants.SERVO_NORM);
    f.set(PizzaBoxConstants.STOP);
  
    return "RESETING ANGLE (NO TOUCHY THE PIZZABOX)";

    
  }
  public static void SpeedyArm(TalonFX m) {
    if (ArmSubsystem.encoderGetAngle() < 99 && ArmSubsystem.encoderGetAngle() > 261) {
      m.set(1);
    }
     else {
      m.set(-0.8);
     }
  }
}