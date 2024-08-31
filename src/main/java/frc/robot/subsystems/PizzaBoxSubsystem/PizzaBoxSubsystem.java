package frc.robot.subsystems.PizzaBoxSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



// COMMANDS NEED TO BE RUN WITH LAMBDAS
public class PizzaBoxSubsystem extends SubsystemBase {
  
  private static TalonFX m_PizzaBoxMotor;
  private Servo PBservo;
  private Servo PBFlapServo;
  public boolean hasNote = false;
  public PizzaBoxSubsystem() {

    m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PIZZA_BOX_ID,PizzaBoxConstants.PIZZA_BOX_CAN);
    PBservo = new Servo(PizzaBoxConstants.SERVO_PWD);
    PBFlapServo = new Servo(PizzaBoxConstants.FLAP_PWD);

  }

  

  public Command spit_command() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(1));
  }

  public Command slurp_command() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(-1));
  }

  public Command stopMotor() {
    return this.run(() -> m_PizzaBoxMotor.set(.00));
  }

  public static double motorSpeed(TalonFX motor) {
    return motor.getRotorVelocity().getValue();
  }

  public Command stopFlap() {
    return this.run(() -> PBFlapServo.set(PizzaBoxConstants.STOP));
  }
  public Command stopKicker(Servo servo) {
    return this.run(() -> PBservo.set(PizzaBoxConstants.STOP));
  }

  public Command speedyArm_Command(DoubleSupplier f) {
    if (f.getAsDouble() > 99 && f.getAsDouble() < 261) {
      return this.run(() -> m_PizzaBoxMotor.set(1));
    }
     else {
      return this.run(() -> m_PizzaBoxMotor.set(-0.8));
     }

  }

  public Command setKicker(double f) {
    return this.run(() -> PBservo.set(f));
  }  

  public Command setFlap(double f) {
    return this.run(() -> PBFlapServo.set(f));
  }


  public double flapAngle(){
    return PBFlapServo.getAngle();
  }

  public double kickerAngle() {

    return PBservo.getAngle();
  }

  public static boolean AtVelocity(double d) {

    if (m_PizzaBoxMotor.getVelocity().getValueAsDouble() == d) {
      return true;
    }
    else {
      return false;
    }
  }
}