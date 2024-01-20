// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.GyroMoment;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class PigeonGyro extends AbstractGyro {

  private WPI_Pigeon2 pigeon;

  // LOOK AT THIS!!!!11
  // imu is weird
  // so getRoll actually returns pitch, getPitch actually returns roll
  // same for the get rate functions

  // TO DO LIST
  // -Fix name of get methods
  // -Make auto balance faster
  // -

  public PigeonGyro(String canivoreName) {
    pigeon = new WPI_Pigeon2(DrivetrainConstants.PIGEON_ID, canivoreName);
  }

  @Override
  public void reset() {
    pigeon.reset();
  }

  @Override
  public void calibrate() {
    pigeon.calibrate();
  }

  @Override
  public double getAngle() {
    double angle = pigeon.getAngle();
    Logger.getInstance().recordOutput("Gyro/Angle", angle);
    return angle;
  }

  @Override
  public double getYaw() {
    double yaw = pigeon.getYaw();
    Logger.getInstance().recordOutput("Gyro/Yaw", yaw);
    return yaw;
  }

  @Override
  public double getPitch() {
    double pitch = pigeon.getPitch();
    Logger.getInstance().recordOutput("Gyro/Pitch", pitch);
    return pitch;
  }

  @Override
  public double getRoll() {
    double roll = pigeon.getRoll();
    Logger.getInstance().recordOutput("Gyro/Roll", roll);
    return roll;
  }

  @Override
  public void configMountPoseRoll(double roll) {
    pigeon.configMountPoseRoll(roll);
  }

  @Override
  public void configMountPoseYaw(double yaw) {
    pigeon.configMountPoseYaw(yaw);
  }

  @Override
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  @Override
  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  @Override
  public double getYawRate() {
    double[] xyz_dps = new double[3];
    pigeon.getRawGyro(xyz_dps);
    Logger.getInstance().recordOutput("Gyro/Yaw Rate", xyz_dps[2]);
    return xyz_dps[2];
  }

  @Override
  public double getPitchRate() {
    double[] xyz_dps = new double[3];
    pigeon.getRawGyro(xyz_dps);
    Logger.getInstance().recordOutput("Gyro/Pitch Rate", xyz_dps[1]);
    return xyz_dps[1];
  }

  @Override
  public double getRollRate() {
    double[] xyz_dps = new double[3];
    pigeon.getRawGyro(xyz_dps);
    Logger.getInstance().recordOutput("Gyro/Roll Rate", xyz_dps[0]);
    return xyz_dps[0];
  }
}
