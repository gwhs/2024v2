package frc.robot.subsystems.PizzaBoxSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;

/*
 * This is the basic idea for a function
 * please follow it so that it is readable at a glance and no hypnoanalyses are needed just to figure out what a function does:
 * public static String <functionname>(TalonFX motor, Servo s, Servo s2) {
 * 
 * <do stuff here>
 * 
 * 
 * return System.out.println(<info message>)
 * }
 * If the function doesn't return a string change the return but you get the idea.
 * 
 * 
 */
public class PizzaBoxFuncs {
public static String reset(Servo s, Servo s2, TalonFX f) {
    s.set(PizzaBoxConstants.stop);
    s2.set(PizzaBoxConstants.stop);
    f.set(PizzaBoxConstants.stop);
  
    return "RESETING ANGLE (NO TOUCHY THE PIZZABOX)";
  }

  public static String Spit(TalonFX motor) {
    motor.set(-1);
    return "SPITTING (DON'T BLOCK THE EXIT)";
  }
  public static String Slurp(TalonFX motor) {
    motor.set(1);
    return "SLURPING (THE PIZZA BOX NEEDS TO LINE UP WITH THE INTAKE)";
  }}