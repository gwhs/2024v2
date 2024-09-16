package frc.robot.subsystems.PizzaBoxSubsystem;

public interface PizzaBoxIO {
    public void setFlap(double speed);  
    public void setMotor(double speed);
    public void setKicker(double speed);
    public boolean atMotorSpeed(double speed);
    public double getFlapAngle();
    public double getKickerAngle();
    public double motorSpeed();
    public void update();
}

 
