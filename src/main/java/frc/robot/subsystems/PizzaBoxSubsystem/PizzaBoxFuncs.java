package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;

/*
 * This is the basic idea for a function in this file,
 * please follow it so that it is readable at a glance and no hypnoanalyses are needed just to figure out what a function does:
 * 
 * public static String <functionname>(TalonFX motor, Servo s, Servo s2) {
 * 
 * <do stuff here>
 * 
 * return <info message>
 * }
 * 
 * If the function doesn't return a string change the return but you get the idea.
 */
  
public class PizzaBoxFuncs {
public static String reset(Servo s, Servo s2, TalonFX f) {
    s.set(PizzaBoxConstants.STOP);
    s2.set(PizzaBoxConstants.STOP);
    f.set(PizzaBoxConstants.STOP);
  
    return "RESETING ANGLE (NO TOUCHY THE PIZZABOX)";
  }

  public static String Spit(TalonFX motor) {
    motor.set(-1);
    return "SPITTING (DON'T BLOCK THE EXIT)";
  }
  public static String Slurp(TalonFX motor) {
    motor.set(1);
    return "SLURPING (THE PIZZA BOX NEEDS TO LINE UP WITH THE INTAKE)";
}

  public static double motorSpeed(TalonFX motor) {

    return m.getRotorVelocity().getValue();
  }

  public static String stopFlap(Servo servo) {
    servo.set(PizzaBoxConstants.STOP);
    return "STOPPING FLAP";
  }

  public static String stopKicker(Servo servo) {
    servo.set(PizzaBoxConstants.STOP);
    return "STOPPING KICKER";
  }
  
  public static String stopMotor(TalonFX motor) {
    motor.set(.00);
    return "STOPPING MOTOR";
  }
}
  