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
    public static final int kOperatorControllerPort = 1;
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
    public static final class Arm {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int ARM_MAX_ANGLE = 270;
    public static final int ARM_MIN_ANGLE = 0;
    public static final int ROTATION_TO_DEGREES = 360;
    public static final int GEAR_RATIO = 16;
    public static final double ENCODER_RAW_TO_ROTATION = 8132.;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_LOWER_INTAKE_ID = 55;
    public static final int INTAKE_SPIN_MOTOR_ID = 20;
    public static final int INTAKE_ENCODER_CHANNEL_ID = 1; 
    public static final int INTAKE_NOTESENSOR_CHANNEL_ID = 7;
    public static final double INTAKE_MOTOR_VELOCITY = 100; 
    public static final double INTAKE_MOTOR_ACCELERATION = 5; 
    public static final double TOLERANCE = 2; // in degrees
    public static final int GEAR_RATIO = 100;
    public static final double NOTE_DELAY = 2; //change accordingly
    public static final double MAX_ARM_ANGLE = 106; //need to check max arm angle
    public static final double ROTATION_TO_DEGREES = 360;
    public static final double ENCODER_RAW_TO_ROTATION = 8132.;
    public static final double ENCODER_OFFSET = 220 + 97; //need to check the encoder value 
  }

  public static final class LEDConstants
  {
    public static final int ledPortNumber = 9;
  }
}



