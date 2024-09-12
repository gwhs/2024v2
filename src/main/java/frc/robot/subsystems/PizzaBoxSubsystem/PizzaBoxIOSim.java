package frc.robot.subsystems.PizzaBoxSubsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class PizzaBoxIOSim implements PizzaBoxIO{


    private FlywheelSim motor = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1, 0.005);
    private PWMSim flap = new PWMSim(1);
    private PWMSim kicker = new PWMSim(0);


    public void setFlap(double speed) {
        flap.setSpeed(speed);
    }

    
    public void setMotor(double speed) {
        motor.setInputVoltage(speed);
    }

    
    public void setKicker(double speed) {
        kicker.setSpeed(speed);
    
    }

    
    public boolean AtMotorSpeed(double speed) {
        if (motor.getAngularVelocityRadPerSec() == speed) {
            return true;
        }
       else{
         return false;
    }
    }

    
    public double getFlapAngle() {
        return flap.getPosition();
    }

    
    public double getKickerAngle() {
        return kicker.getPosition();
    }


    
    public void update() {
       motor.update(.020);
       
    }

    
    
}