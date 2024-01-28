// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
     public static final int kDriverControllerPort = 0;
  }

  // move this else where create a data object
  public static final class LimeLightConstants {

    // for ryker
    public static final double MAX_LIMELIGHT_ERROR_DEGREES =
        1; // limelight max degrees off, max degrees error
    public static final double CAMERA_HEIGHT = 84.5;
    public static final double TARGET_HEIGHT = 61;
    public static final double MOUNTING_ANGLE = -10;
    public static final double LOWER_DISTANCE_SHOOT = 114;
    public static final double MID_DISTANCE_SHOOT = 76;
    public static final double TOP_DISTANCE_SHOOT = 42;
  }

  public static final class ClimbConstants {
    //might need to cahnge these 4
  public static final double CLIMBER_RATIO = 36;
  public static final int TICKS_PER_REVOLUTION = 2048;
  public static final double INCHES_PER_REVOLUTION = 4.55;
  public static final double PITCH_DIAMETER_30 = 1.5;

  public static final double CLIMB_DISTANCE = 0;
  public static final double CLIMB_MOTOR_SPEED = 0;

  public static final int MOTOR_LEFT_ID = 0;
  public static final boolean MOTOR_LEFT_INVERTED = false;
  public static final boolean MOTOR_RIGHT_INVERTED = false;
  public static final int MOTOR_RIGHT_ID = 0;
  }
  
}
