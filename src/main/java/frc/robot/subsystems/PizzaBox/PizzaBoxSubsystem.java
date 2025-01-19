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

    SmartDashboard.putData("Command Testing/Pizza box Spit", spit_command(.69));
    SmartDashboard.putData("Command Testing/Pizza box Slurp", slurp_command(.69));
    SmartDashboard.putData("Command Testing/Stop Pizza box", stopMotor());
    SmartDashboard.putData("Command Testing/Reset Flap", stopFlap());
    SmartDashboard.putData("Command Testing/Reset Kicker", stopKicker());
    SmartDashboard.putData("Command Testing/Deploy Flap", setFlap());
    SmartDashboard.putData("Command Testing/Extend Kicker", setKicker());
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
    return this.runOnce(() -> pizzaBoxIO.setMotor(0))
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
    return this.run(() -> {
      if(f.getAsDouble() <= 100 || f.getAsDouble() >= 270) {
        pizzaBoxIO.setMotor(-0.05);
      }
      else {
        pizzaBoxIO.setMotor(1);
      }
    });

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

  public Boolean atVelocity(double d) {

    if (pizzaBoxIO.atMotorSpeed(d)) {
      return true;
    } else {
      return false;
    }
  }

  public double getVelocity() {
    return pizzaBoxIO.getMotorSpeed();
  }

  @Override
  public void periodic() {
    NetworkTableInstance.getDefault().getEntry("PizzaBox: Motor Speed").setNumber(pizzaBoxIO.getMotorSpeed());
    pizzaBoxIO.update();
  }
}