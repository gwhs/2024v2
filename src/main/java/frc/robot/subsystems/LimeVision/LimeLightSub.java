// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Improved upon TechSupport's file

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class LimeLightSub extends SubsystemBase {

  // set up a new instance of NetworkTables (the api/library used to read values from limelight)
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

  // return network table values for tx and ty using getEntry()
  NetworkTableEntry tv =
      networkTable.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry tx =
      networkTable.getEntry("tx"); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees |
  // LL2: -29.8 to 29.8 degrees)
  NetworkTableEntry ty =
      networkTable.getEntry("ty"); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees |
  // LL2: -24.85 to 24.85 degrees)
  NetworkTableEntry ta = networkTable.getEntry("ta"); // Target Area (0% of image to 100% of image)
  NetworkTableEntry ts = networkTable.getEntry("ts"); // Skew or rotation (-90 degrees to 0 degrees)
  NetworkTableEntry pipe = networkTable.getEntry("getpipe");

  NetworkTableEntry tid = networkTable.getEntry("tid"); // gets apriltag id

  NetworkTableEntry botPoseBlue = networkTable.getEntry("botpose_wpiblue"); // botpose mega
  NetworkTableEntry botPoseRed = networkTable.getEntry("botpose_wpibred"); // botpose mega

  // may be useful later
  private double kCameraHeight =
      LimeLightConstants.CAMERA_HEIGHT; // LimelightConstants.kCameraHeight;
  private double kTargetHeight =
      LimeLightConstants.TARGET_HEIGHT; // LimelightConstants.kTargetHeight;

  private LimeLightComms limelight_comm;

  /** Creates a new LimeLightSub. */
  public LimeLightSub(String limelight_networktable_name) {
    limelight_comm = new LimeLightComms(limelight_networktable_name);
    limelight_comm.set_entry_double("ledMode", 3);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    SmartDashboard.putNumber("theta", getTheta());
    SmartDashboard.putNumber("AngleToTarget", getAngle());
  }

  public boolean hasTarget() {
    return tv.getDouble(0) >= .9;
  }

  public double getTx() {
    double Tx = tx.getDouble(0);
    return Tx;
  }

  public int getID() {
    int id = (int) tid.getDouble(-1) - 1;
    return id;
  }

  public double getTy() {
    double Ty = ty.getDouble(0);
    return Ty;
  }

  public double getTa() {
    double Ta = ta.getDouble(0);
    return Ta;
  }

  public double getTheta() {
    double Theta = getTy() + LimeLightConstants.MOUNTING_ANGLE;
    return Theta;
  }

  public double getAngle() {
    double angle = Math.toRadians(getTx());
    return angle;
  }

  public double getPipeline() {
    double Pipeline = limelight_comm.get_entry_double("pipeline");
    return Pipeline;
  }

  public void setPipeline(double pipeline) {
    limelight_comm.set_entry_number("pipeline", pipeline);
  }

  public boolean checkPipe() {
    return !(limelight_comm.get_entry_double("pipeline") < .5);
  }

  // ta error from expected ta size
  public double getTaDistance() {
    return 85.0 - (getTa() * 100);
  }
}