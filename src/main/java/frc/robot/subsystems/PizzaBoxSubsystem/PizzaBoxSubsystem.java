package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// COMMANDS NEED TO BE RUN WITH LAMBDAS
public class PizzaBoxSubsystem extends SubsystemBase {
  
  private TalonFX m_PizzaBoxMotor;
  private Servo PBservo;
  private Servo PBFlapServo;
  public PizzaBoxSubsystem() {
    m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PizzaBoxID,PizzaBoxConstants.PizzaBoxCAN);
    PBservo = new Servo(PizzaBoxConstants.servoPWD);
    PBFlapServo = new Servo(PizzaBoxConstants.flapPWD);

  }

  public Command reset_command(Servo s, Servo s2, TalonFX f) {
    return this.runOnce(() -> PizzaBoxFuncs.reset(PBservo,PBFlapServo,m_PizzaBoxMotor));
  }

  public Command spit_command(TalonFX m) {
    return this.runOnce(() -> PizzaBoxFuncs.Spit(m));
  }

  public Command slurp_command(TalonFX m) {
    return this.runOnce(() -> PizzaBoxFuncs.Slurp(m));
  }
}