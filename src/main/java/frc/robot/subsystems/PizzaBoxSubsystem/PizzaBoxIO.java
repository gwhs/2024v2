package frc.robot.subsystems.PizzaBoxSubsystem;

public interface PizzaBoxIO {
    public void setFlap(double speed);  
    public void setMotor(double speed);
    public void setKicker(double speed);
    public boolean AtMotorSpeed(double speed);
    public double getFlapAngle();
    public double getKickerAngle();
    public void update();
}

 
