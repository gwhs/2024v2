package frc.robot.subsystems.PizzaBox;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PizzaBoxSubsystem extends SubsystemBase {
  private final PizzaBoxIO pizzaBoxIO;
  public boolean hasNote = false;

  public PizzaBoxSubsystem() {
    if (RobotBase.isSimulation()) {
      pizzaBoxIO = new PizzaBoxIOSim();
    } else {
      pizzaBoxIO = new PizzaBoxIOReal();
    }

    SmartDashboard.putData(spit_command(.69));
    SmartDashboard.putData(slurp_command(.69));
    SmartDashboard.putData(stopMotor());
    SmartDashboard.putData(stopFlap());
    SmartDashboard.putData(stopKicker());
    SmartDashboard.putData(setFlap());
    SmartDashboard.putData(setKicker());
    SmartDashboard.putData(speedyArm_Command(() -> 50));
  }

  public double getSpeed() {
    return pizzaBoxIO.motorSpeed();
  }

  public Command spit_command(double speed) {
    return this.runOnce(() -> pizzaBoxIO.setMotor(speed))
        .withName("Spit");
  }

  public Command slurp_command(double speed) {
    return this.runOnce(() -> pizzaBoxIO.setMotor(-speed))
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

    } else {
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

  public double flapAngle() {
    return pizzaBoxIO.getFlapAngle();
  }

  public double kickerAngle() {

    return pizzaBoxIO.getKickerAngle();
  }

  public boolean atVelocity(double d) {

    if (pizzaBoxIO.atMotorSpeed(d)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    NetworkTableInstance.getDefault().getEntry("PizzaBox: Motor Speed").setNumber(pizzaBoxIO.motorSpeed());
    pizzaBoxIO.update();
  }
}