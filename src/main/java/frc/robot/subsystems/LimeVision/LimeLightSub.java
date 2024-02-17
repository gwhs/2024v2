// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Improved upon TechSupport's file

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class LimeLightSub extends SubsystemBase {

  // PID constants
  private final double kP = 0.04;
  private final double kD = 0;
  private final double kI = 0;
  // Set target point
  private static double setPoint = 0;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  PIDController PIDVision = new PIDController(kP, kI, kD);

  // in meters
  double[][] apriltag = {{15.08,0.24,1.35},
                         {16.18,0.89,1.35},
                         {16.58,4.98,1.45},
                         {16.58,5.55,1.45},
                         {14.70,8.23,1.35},
                         {1.84,8.23,1.35},
                         {-0.04,5.55,1.45},
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
    
    // setting target point for PID
    PIDVision.setSetpoint(setPoint);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    SmartDashboard.putNumber("theta", getTheta());
    SmartDashboard.putNumber("AngleToTarget", getAngle());

    SmartDashboard.putNumber("Distance", getDistance()); // printing

    System.out.println("Theta: " + aprilTagFieldLayout);
    
    // displaying error values
    SmartDashboard.putNumber("Error Angle", getError());

    // This method will be called once per scheduler run
    double currTx = limelight_comm.get_entry_double("tx");
    SmartDashboard.putNumber("tx", currTx);
    // System.out.println(currTx);
  }

  public boolean hasTarget() {
    return tv.getDouble(0) >= .9;
  }

  public double getTx() {
    double Tx = tx.getDouble(0);
    return Tx;
  }

  public double getID() {
    double id = tid.getDouble(0);
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

  // calculates error from target
  public double getError() {
    return PIDVision.calculate(getTx());
  }

  // // calculates distance from tag --- not that good
  // public double getDistance() {
  //   double distance = (LimeLightConstants.TARGET_HEIGHT - LimeLightConstants.CAMERA_HEIGHT) / (Math.tan(getTheta()));
  //   return distance;
  // }

  // using distance formula (relative to field)
  public double getDistance() {
    double[] botPose = botPoseBlue.getDoubleArray(new double[6]); // x,y,z,rx,ry,rz

    double distance = Math.sqrt(Math.pow((apriltag[(int) getID()][0]) - botPose[0], 2) + Math.pow((apriltag[(int) getID()][1]) - botPose[1], 2)); 

    distance /= 0.0256;
    return distance;
  }

  // setsPoint PID
  public void setPoint(double target) {
    PIDVision.setSetpoint(target);
  }

  public double getDistanceTx() {
    double error = PIDVision.calculate(getTx());
    return error;
  }

}