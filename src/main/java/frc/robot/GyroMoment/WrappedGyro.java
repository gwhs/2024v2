// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

/** Add your docs here. */
public class WrappedGyro {

  private AbstractGyro gyro;

  public enum GyroType {
    PIGEON,
  }

  public WrappedGyro(GyroType type, String canivoreName) {
    if (Robot.isReal()) {
      if (type == GyroType.PIGEON) {
        gyro = new PigeonGyro(canivoreName);
      }
    }
  }

  public void reset() {
    gyro.reset();
  }

  public void calibrate() {
    gyro.calibrate();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public void configMountPoseRoll(double roll) {
    gyro.configMountPoseRoll(roll);
  }

  public void configMountPoseYaw(double yaw) {
    gyro.configMountPoseYaw(yaw);
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  public double getYawRate() {
    return gyro.getYawRate();
  }

  public double getPitchRate() {
    return gyro.getPitchRate();
  }

  public double getRollRate() {
    return gyro.getRollRate();
  }
}
