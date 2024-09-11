package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class PizzaBoxIOReal implements PizzaBoxIO {
    private TalonFX m_PizzaBoxMotor = new TalonFX(PizzaBoxConstants.PIZZA_BOX_ID,PizzaBoxConstants.PIZZA_BOX_CAN);
    private Servo PBservo = new Servo(PizzaBoxConstants.SERVO_PWD);
    private Servo PBFlapServo = new Servo(PizzaBoxConstants.FLAP_PWD);

    public void setMotor(double speed) {
        m_PizzaBoxMotor.set(speed);
    }
    public void setFlap(double speed) {
        PBFlapServo.set(speed);
    }
    public void setKicker(double speed) {
        PBservo.set(speed);
    }
    
    
    
    
    public boolean MotorSpeed(double speed) {

        if(m_PizzaBoxMotor.getVelocity().getValueAsDouble() == speed) {
            return true;
        }
        return false;
        
    }

    public double getFlapAngle() {
       return PBFlapServo.getAngle();
    }

    public double getKickerAngle() {
       return PBservo.getAngle();
    }
    
}