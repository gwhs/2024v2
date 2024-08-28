package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// COMMANDS NEED TO BE RUN WITH LAMBDAS
public class PizzaBoxSubsystem extends SubsystemBase {
  
  private TalonFX m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PizzaBoxID,PizzaBoxConstants.PizzaBoxCAN);
  private Servo PBservo;
  private Servo PBFlapServo;
  public PizzaBoxSubsystem() {
    m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PizzaBoxID,PizzaBoxConstants.PizzaBoxCAN);
    PBservo = new Servo(PizzaBoxConstants.servoPWD);
    PBFlapServo = new Servo(PizzaBoxConstants.flapPWD);

  }

  

  @Override
  public void periodic() {
    
  }

  public String reset(Servo s, Servo s2, TalonFX f) {
    s.set(PizzaBoxConstants.stop);
    s2.set(PizzaBoxConstants.stop);
    f.set(PizzaBoxConstants.stop);
  
    return "RESETING ANGLE (NO TOUCHY THE PIZZABOX)";
  }

  public Command reset_command(Servo s, Servo s2, TalonFX f) {
    s.set(PizzaBoxConstants.stop);
    s2.set(PizzaBoxConstants.stop);
    f.set(PizzaBoxConstants.stop);

    return this.runOnce(() -> reset(PBservo,PBFlapServo,m_PizzaBoxMotor));
  }
}
