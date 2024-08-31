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

  

  public Command spit_command(TalonFX motor) {
    return this.runOnce(() -> m_PizzaBoxMotor.set(1));
  }

  public Command slurp_command() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(-1));
  }

  public Command stopMotor(TalonFX motor) {
    return this.run(() -> motor.set(.00));
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

  public Command speedyArm_Command() {
    if (ArmSubsystem.encoderGetAngle() > 99 && ArmSubsystem.encoderGetAngle() < 261) {
      return this.run(() -> m_PizzaBoxMotor.set(1));
    }
     else {
      return this.run(() -> m_PizzaBoxMotor.set(-0.8));
     }

  }

  

}