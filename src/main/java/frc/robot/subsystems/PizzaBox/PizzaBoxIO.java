package frc.robot.subsystems.PizzaBox;

public interface PizzaBoxIO {
  public void setFlap(double angle);

  public void setMotor(double speed);

  public void setKicker(double angle);

  public boolean atMotorSpeed(double speed);

  public double getFlapAngle();

  public double getKickerAngle();

  public double motorSpeed();

  public void update();
}
