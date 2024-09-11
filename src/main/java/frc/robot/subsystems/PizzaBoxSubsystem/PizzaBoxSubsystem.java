package frc.robot.subsystems.PizzaBoxSubsystem;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



// COMMANDS NEED TO BE RUN WITH LAMBDAS
public class PizzaBoxSubsystem extends SubsystemBase {
  
  private TalonFX m_PizzaBoxMotor;
  private Servo PBservo;
  private Servo PBFlapServo;
  public boolean hasNote = false;
  public PizzaBoxSubsystem() {

    m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PIZZA_BOX_ID,PizzaBoxConstants.PIZZA_BOX_CAN);
    PBservo = new Servo(PizzaBoxConstants.SERVO_PWD);
    PBFlapServo = new Servo(PizzaBoxConstants.FLAP_PWD);
    


    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout pizzaCommandsLayout = tab.getLayout("PizzaBox Commands", BuiltInLayouts.kList)
      .withSize(2,2)
      .withProperties(Map.of("Label Position", "HIDDEN"));

    pizzaCommandsLayout.add(spit_command());
    pizzaCommandsLayout.add(slurp_command());
    pizzaCommandsLayout.add(stopMotor());
    pizzaCommandsLayout.add(stopFlap());
    pizzaCommandsLayout.add(stopKicker());
    pizzaCommandsLayout.add(setFlap());
    pizzaCommandsLayout.add(setKicker());
    pizzaCommandsLayout.add(speedyArm_Command(() -> 50));
  }

  

  public Command spit_command() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(1))
    .withName("Spit");
  }

  public Command slurp_command() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(-1))
    .withName("Slurp");
  }

  public Command stopMotor() {
    return this.runOnce(() -> m_PizzaBoxMotor.set(.00))
    .withName("STOP MOTOR");
  }

  
  public Command stopFlap() {
    return this.runOnce(() -> PBFlapServo.set(PizzaBoxConstants.RESET_FLAP))
    .withName("STOP FLAP");
  }
  public Command stopKicker() {
    return this.runOnce(() -> PBservo.set(PizzaBoxConstants.RESET_KICKER))
    .withName("STOP KICKER");
  }

  public Command speedyArm_Command(DoubleSupplier f) {
    if (f.getAsDouble() > 99 && f.getAsDouble() < 261) {
      return this.runOnce(() -> m_PizzaBoxMotor.set(1))
      .withName("GAS GAS GAS");
      
    }
     else {
      return this.runOnce(() -> m_PizzaBoxMotor.set(-0.8))
      .withName("!GAS GAS GAS");
     }

  }

  public Command setKicker() {
    return this.runOnce(() -> PBservo.set(180))
    .withName("SET KICKER");
  }  

  public Command setFlap() {
    return this.runOnce(() -> PBFlapServo.set(200))
    .withName("SET FLAP");
  }


  public double flapAngle(){
    return PBFlapServo.getAngle();
  }

  public double kickerAngle() {

    return PBservo.getAngle();
  }

  public boolean atVelocity(double d) {

    if (m_PizzaBoxMotor.getVelocity().getValueAsDouble() == d) {
      return true;
    }
    else {
      return false;
    }
  }
}