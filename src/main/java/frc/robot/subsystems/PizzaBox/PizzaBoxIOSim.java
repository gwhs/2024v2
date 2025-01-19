package frc.robot.subsystems.PizzaBox;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class PizzaBoxIOSim implements PizzaBoxIO {

  private FlywheelSim motor = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
      DCMotor.getKrakenX60Foc(1));
  private PWMSim flap = new PWMSim(1);
  private PWMSim kicker = new PWMSim(0);

  public void setFlap(double angle) {
    flap.setSpeed(angle);
  }

  public void setMotor(double speed) {
    motor.setInputVoltage(speed * 11);
  }

  public void setKicker(double angle) {
    kicker.setSpeed(angle);

  }

  public boolean atMotorSpeed(double speed) {
    if (motor.getAngularVelocityRadPerSec() >= speed) {
      return true;
    } else {
      return false;
    }
  }

  public double getFlapAngle() {
    return flap.getPosition();
  }

  public double getKickerAngle() {
    return kicker.getPosition();
  }

  public double motorSpeed() {
    return motor.getAngularVelocity().in(RotationsPerSecond);
  }

  public void update() {
    motor.update(.020);

  }

  

}