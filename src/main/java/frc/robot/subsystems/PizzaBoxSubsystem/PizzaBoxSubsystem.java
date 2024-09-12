package frc.robot.subsystems.PizzaBoxSubsystem;

import java.util.Map;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PizzaBoxSubsystem extends SubsystemBase {
  private final PizzaBoxIO pizzaBoxIO;
  public boolean hasNote = false;


  public PizzaBoxSubsystem() {
    if(RobotBase.isSimulation()) {
      pizzaBoxIO = new PizzaBoxIOSim();
    }
    else {
      pizzaBoxIO = new PizzaBoxIOReal();
    }
    
    


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
    return this.runOnce(() -> pizzaBoxIO.setMotor(1))
    .withName("Spit");
  }

  public Command slurp_command() {
    return this.runOnce(() -> pizzaBoxIO.setMotor(-1))
    .withName("Slurp");
  }

  public Command stopMotor() {
    return this.runOnce(() -> pizzaBoxIO.setMotor(.00))
    .withName("STOP MOTOR");
  }

  
  public Command stopFlap() {
    return this.runOnce(() -> pizzaBoxIO.setFlap(PizzaBoxConstants.RESET_FLAP))
    .withName("STOP FLAP");
  }
  public Command stopKicker() {
    return this.runOnce(() -> pizzaBoxIO.setKicker(PizzaBoxConstants.RESET_KICKER))
    .withName("STOP KICKER");
  }

  public Command speedyArm_Command(DoubleSupplier f) {
    if (f.getAsDouble() > 99 && f.getAsDouble() < 261) {
      return this.runOnce(() -> pizzaBoxIO.setMotor(1))
      .withName("GAS GAS GAS");
      
    }
     else {
      return this.runOnce(() -> pizzaBoxIO.setMotor(-0.8))
      .withName("!GAS GAS GAS");
     }

  }

  public Command setKicker() {
    return this.runOnce(() -> pizzaBoxIO.setKicker(PizzaBoxConstants.KICKER_OUT))
    .withName("SET KICKER");
  }  

  public Command setFlap() {
    return this.runOnce(() -> pizzaBoxIO.setFlap(PizzaBoxConstants.FLAP_OUT))
    .withName("SET FLAP");
  }


  public double flapAngle(){
    return pizzaBoxIO.getFlapAngle();
  }

  public double kickerAngle() {

    return pizzaBoxIO.getKickerAngle();
  }

  public boolean atVelocity(double d) {

    if (pizzaBoxIO.AtMotorSpeed(d)) {
      return true;
    }
    else {
      return false;
    }
  }
}