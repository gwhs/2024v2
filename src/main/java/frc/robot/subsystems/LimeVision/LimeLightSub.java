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

  // PID constants
  private final double kPX = 0.8;
  private final double kDX = 0.1;
  private final double kIX = 0;

  private final double kPY = 0.6;
  private final double kDY = 0.1;
  private final double kIY = 0;

  /* PID constants for faceAprilTag
   * P = 0.04, D = 0, I = 0
   */
  private final double kPTheta = 0.04;
  private final double kDTheta = 0;
  private final double kITheta = 0;
  // Set target point

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  PIDController PIDVisionX = new PIDController(kPX, kIX, kDX);
  PIDController PIDVisionY = new PIDController(kPY, kIY, kDY);
  PIDController PIDVisionTheta = new PIDController(kPTheta, kITheta, kDTheta);

  // in meters
  double[][] apriltag = {{15.08,0.24,1.35},
                         {16.18,0.89,1.35},
                         {16.58,4.98,1.45},
                         {16.58,5.55,1.45},
                         {14.70,8.23,1.35},
                         {1.84,8.23,1.35},
                         {-0.04,5.38,1.45},
                         {-0.04,4.98,1.45},
                         {0.36,0.88,1.35},
                         {1.46,0.24,1.35},
                         {11.90,3.71,1.32},
                         {11.90,4.50,1.32},
                         {11.22,4.10,1.32},
                         {5.32,4.10,1.32},
                         {4.64,4.50,1.32},
                         {4.64,3.71,1.32}};

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

    // distance
    Shuffleboard.getTab("Limelight").addNumber("Distance X", ()-> getDistanceX());
    Shuffleboard.getTab("Limelight").addNumber("Distance Y", ()-> getDistanceY());
    Shuffleboard.getTab("Limelight").addNumber("Distance X Error", ()-> getErrorX());
    Shuffleboard.getTab("Limelight").addNumber("Distance Y Error", ()-> getErrorY());
    Shuffleboard.getTab("Limelight").addNumber("Tx", ()-> getTx());
    Shuffleboard.getTab("Limelight").addNumber("Theta Error", ()-> getThetaError());
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

  // setsPoint PID
  public void setPoint(double target, String orientation) {
    if (orientation.toLowerCase().equals("x")) {
      PIDVisionX.setSetpoint(target);
    } else if (orientation.toLowerCase().equals("y")) {
      PIDVisionY.setSetpoint(target);
    } else if (orientation.toLowerCase().equals("theta")) {
      PIDVisionTheta.setSetpoint(target);
    }
  }

  // using distance formula (relative to field) calculates distance from tag
  public double getDistance() {
    double distance = -1;
    if (getID() >= 0) {
    double[] botPose = botPoseBlue.getDoubleArray(new double[7]); // x,y,z,rx,ry,rz
    distance = Math.sqrt(Math.pow((apriltag[getID()][0] - botPose[0]), 2) + Math.pow((apriltag[getID()][1] - botPose[1]), 2)); 
    }
    return distance;
  }

  public double getDistanceX() {
    double distance = -1;
    if (getID() >= 0) {
    double[] botPose = botPoseBlue.getDoubleArray(new double[7]); // x,y,z,rx,ry,rz
    distance = apriltag[getID()][0] - botPose[0]; 
    }
    return distance;
  }

  public double getDistanceY() {
    double distance = -1;
    if (getID() >= 0) {
    double[] botPose = botPoseBlue.getDoubleArray(new double[7]); // x,y,z,rx,ry,rz
    distance = apriltag[getID()][1] - botPose[1]; 
    }
    return distance;
  }

  public double getErrorX() {  // front and back
    return PIDVisionX.calculate(getDistanceX());
  }
  
  public double getErrorY() {  // left and right
    return PIDVisionY.calculate(getDistanceY());
  }

  public double getThetaError() {
    return PIDVisionTheta.calculate(getTx());
  }

  // uses 3d botpose instead of 2d values, testing for smooth simultaneous movement, need to test
  public double getSmoothThetaError() {
    double angle = 0;
    if (getID() >= 0) {
    double[] botPose = botPoseBlue.getDoubleArray(new double[7]); // x,y,z,rx,ry,rz
    angle = 180d - botPose[5]; // rz 180 is constant means that facing tag
    }
    return PIDVisionTheta.calculate(angle);
  }

}