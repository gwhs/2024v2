// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbSubsystem;

/** Add your docs here. */
public class ClimbConstants {
  public static final double CLIMBER_RATIO = 36;

  public static final int LEFT_CLIMB_MOTOR_ID = 21;
  public static final boolean LEFT_CLIMB_MOTOR_INVERTED = false;
  public static final boolean RIGHT_CLIMB_MOTOR_INVERTED = true;
  public static final int RIGHT_CLIMB_MOTOR_ID = 14;

  public static final int BOT_RIGHT_LIMIT_ID = 2;
  public static final int BOT_LEFT_LIMIT_ID = 3;
  public static final int TOP_RIGHT_LIMIT_ID = 4;
  public static final int TOP_LEFT_LIMIT_ID = 5;

  public static final double CLIMB_PID_KP = 0.5;
  public static final double CLIMB_PID_KI = 0;
  public static final double CLIMB_PID_KD = 0;

  public static final double MAX_ACCELERATION = 150.0;
  public static final double MAX_VELOCITY = 300.0;

  public static final double LEFT_UP_POSITION = -193.94;
  public static final double LEFT_DOWN_POSITION = -12.3;
  public static final double RIGHT_UP_POSITION = 193.4;
  public static final double RIGHT_DOWN_POSITION = 12.3;

}