// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Improved upon TechSupport's file

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimeLightSub extends SubsystemBase {

  // PID constants
  private final double kP = 0.5;
  private final double kD = 0.2;
  private final double kI = 0;

  private PIDController PIDVision = new PIDController(kP, kI, kD);
  private PIDController PIDVisionY = new PIDController(kP, kI, kD);
  private PIDController PIDVisionTheta = new PIDController(kP, kI, kD);
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

  // botpose megatag
  NetworkTableEntry botpose = networkTable.getEntry("botpose");
  NetworkTableEntry blueBotPose = networkTable.getEntry("botpose_wpiblue");
  NetworkTableEntry targetSpace = networkTable.getEntry("botpose_targetspace");
  NetworkTableEntry tid = networkTable.getEntry("tid");
  

  

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
    Shuffleboard.getTab("Limelight").addDouble("BotPose TX", ()->getBlueBotPose()[0]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose TY", ()->getBlueBotPose()[1]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose TZ", ()->getBlueBotPose()[2]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose RX", ()->getBlueBotPose()[3]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose RY", ()->getBlueBotPose()[4]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose RZ", ()->getBlueBotPose()[5]);
    Shuffleboard.getTab("Limelight").addDouble("BotPose ms", ()->getBlueBotPose()[6]);
    Shuffleboard.getTab("Limelight").addDouble("ID", ()->getBlueBotPose()[7]);
    Shuffleboard.getTab("Limelight").addDouble("Distance", ()->getBlueBotPose()[8]);

    Shuffleboard.getTab("Limelight").addDouble("TargetPose TX", ()->getTargetSpace()[0]);
    Shuffleboard.getTab("Limelight").addDouble("TargetPose TY", ()->getTargetSpace()[1]);
    Shuffleboard.getTab("Limelight").addDouble("TargetPose TZ", ()->getTargetSpace()[2]);
    Shuffleboard.getTab("Limelight").addDouble("TargetPose RX", ()->getTargetSpace()[3]);
    Shuffleboard.getTab("Limelight").addDouble("TargetPose RY", ()->getTargetSpace()[4]);
    Shuffleboard.getTab("Limelight").addDouble("TargetPose RZ", ()->getTargetSpace()[5]);

  
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    // SmartDashboard.putNumber("theta", getTheta());
    // SmartDashboard.putNumber("AngleToTarget", getAngle());
    
    // // displaying error values
    // SmartDashboard.putNumber("Error Angle", getError());
    SmartDashboard.putNumber("DistanceX Error X", getErrorFromMegaTagX());
    SmartDashboard.putNumber("DistanceX Error Y", getErrorFromMegaTagY());

    // botpose
    if (hasTarget())
    {
    // System.out.println(getBotPose()[0]);

    // SmartDashboard.putNumber("BotPose X", getBlueBotPose()[0]);
    // SmartDashboard.putNumber("BotPose Y", getBlueBotPose()[1]);
    // SmartDashboard.putNumber("BotPose Z", getBlueBotPose()[2]);
    // SmartDashboard.putNumber("BotPose RX", getBlueBotPose()[3]);
    // SmartDashboard.putNumber("BotPose RY", getBlueBotPose()[4]);
    // SmartDashboard.putNumber("BotPose RZ", getBlueBotPose()[5]);

    // SmartDashboard.putNumber("TargetPose X", getTargetSpace()[0]);
    // SmartDashboard.putNumber("TargetPose Y", getTargetSpace()[1]);
    // SmartDashboard.putNumber("TargetPose Z", getTargetSpace()[2]);
    // SmartDashboard.putNumber("TargetPose RX", getTargetSpace()[3]);
    // SmartDashboard.putNumber("TargetPose RY", getTargetSpace()[4]);
    // SmartDashboard.putNumber("TargetPose RZ", getTargetSpace()[5]);
    
    }
    // SmartDashboard.putNumber("BotPose Something", getBotPose()[6]);

    // This method will be called once per scheduler run
    // double currTx = limelight_comm.get_entry_double("tx");
    // SmartDashboard.putNumber("tx", currTx);
    // System.out.println(currTx);
  }

  public boolean hasTarget() {
    return tv.getDouble(0) >= .9;
  }

  public double getTx() {
    double Tx = tx.getDouble(0);
    return Tx;
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

  public double[] getBotPose() {
    double[] botPose = null;
    //SmartDashboard.putBoolean("Limelight Inititialized", isInitialized());
    // if (hasTarget()) {
      botPose = botpose.getDoubleArray(new double[7]);
    // }
    return botPose;
  }

  public double getPoseX() {
    return getBotPose()[0];
  }

  public double getPoseY() {
    return getBotPose()[1];
  }

  public double getThetaZ() {
    return Math.abs(getBotPose()[5]);
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

  // calculates x-distance error
  public double getErrorFromMegaTagX() {
    return PIDVision.calculate(getPoseX());
  }

  // calculates y-distance error
  public double getErrorFromMegaTagY() {
    return PIDVisionY.calculate(getPoseY());
  }

  // calculates y-distance error
  public double getThetaErrorFromMegaTag() {
    return PIDVisionTheta.calculate(getThetaZ());
  }

  public void setSetpointX(double point) {
    PIDVision.setSetpoint(point);
  }

  public void setSetpointY(double point) {
    PIDVisionY.setSetpoint(point);
  }

  public void setSetpointTheta(double point) {
    PIDVisionTheta.setSetpoint(point);
  }

// megatag tx and ty, units in meters
  public Pose2d getPose2D() {
    return new Pose2d(Math.abs(getBotPose()[0]), Math.abs(getBotPose()[1]), Rotation2d.fromDegrees(getBotPose()[5]));
  }

  public double[] getBlueBotPose(){
    double[] blueBotPose = this.blueBotPose.getDoubleArray(new double[9]);

    blueBotPose[6] = getTID();
    blueBotPose[8] = Math.sqrt(Math.pow(apriltag[getTID()+1][0]-blueBotPose[0] ,2) + 
                               Math.pow(apriltag[getTID()+1][1]-blueBotPose[1] ,2));

    return blueBotPose; 
  }

  public double[] getTargetSpace(){
    double[] targetSpace = this.targetSpace.getDoubleArray(new double[6]);
    return targetSpace;
  } 

  public int getTID(){
    int tid = (int)this.tid.getDouble(-1);
    return tid;
  }

}