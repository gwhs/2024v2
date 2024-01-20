// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public abstract class AbstractGyro {

  public abstract void reset();

  public abstract void calibrate();

  public abstract double getAngle();

  public abstract double getYaw();

  public abstract double getPitch();

  public abstract double getRoll();

  public abstract void configMountPoseRoll(double roll);

  public abstract void configMountPoseYaw(double yaw);

  public abstract Rotation2d getRotation2d();

  public abstract void setYaw(double yaw);

  public abstract double getYawRate();

  public abstract double getPitchRate();

  public abstract double getRollRate();
}
