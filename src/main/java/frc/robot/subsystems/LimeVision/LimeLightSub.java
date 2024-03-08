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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.testcontainers.VisionContainer;
import swervelib.SwerveDrive;

public class LimeLightSub extends SubsystemBase {

  // PID constants
  private final double kP = 0.5;
  private final double kD = 0.2;
  private final double kI = 0;
  // private static double distanceFromAprilTag = 0;
  public static Pose2d currentPose;

  public boolean cameraMode = false;

  
  

  private PIDController PIDVision = new PIDController(kP, kI, kD);
  private PIDController PIDVisionY = new PIDController(kP, kI, kD);
  private PIDController PIDVisionTheta = new PIDController(kP, kI, kD);

  



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
  
  

  boolean verbose = true;//If we want to print values
  public boolean wantData = true;//If we want to accept limelight post esitmator

  // may be useful later
  private double kCameraHeight =
      LimeLightConstants.CAMERA_HEIGHT; // LimelightConstants.kCameraHeight;
  private double kTargetHeight =
      LimeLightConstants.TARGET_HEIGHT; // LimelightConstants.kTargetHeight;

  private LimeLightComms limelight_comm;
  private SwerveSubsystem drivebase;

  /** Creates a new LimeLightSub. */
  public LimeLightSub(String limelight_networktable_name, SwerveSubsystem drivebase) {
    limelight_comm = new LimeLightComms(limelight_networktable_name);
    limelight_comm.set_entry_double("ledMode", 3);
    if(verbose){
      Shuffleboard.getTab("Limelight").addDouble("BotPose TX", ()->getBlueBotPose()[0]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose TY", ()->getBlueBotPose()[1]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose TZ", ()->getBlueBotPose()[2]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose RX", ()->getBlueBotPose()[3]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose RY", ()->getBlueBotPose()[4]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose RZ", ()->getBlueBotPose()[5]);
      Shuffleboard.getTab("Limelight").addDouble("BotPose ms", ()->getBlueBotPose()[6]);
      Shuffleboard.getTab("Limelight").addDouble("Tag Count", ()->getBlueBotPose()[7]);
      Shuffleboard.getTab("Limelight").addDouble("Tag Span", ()->getBlueBotPose()[8]);
      Shuffleboard.getTab("Limelight").addDouble("Average Distance", ()->getBlueBotPose()[9]);
      Shuffleboard.getTab("Limelight").addDouble("Average Area", ()->getBlueBotPose()[10]);
      Shuffleboard.getTab("Limelight").addDouble("Tag ID", ()->getBlueBotPose()[11]);
    }
    
    this.drivebase = drivebase;
    
  }

  @Override
  public void periodic() {

  if(verbose){      
    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    
  }
  
  if(wantData){
    setData();
  }
    // This method will be called once per scheduler run
    double currTx = limelight_comm.get_entry_double("tx");
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


  public double[] getBlueBotPose(){
    double[] blueBotPose = this.blueBotPose.getDoubleArray(new double[11]);
    double[] colorBotPose = new double[12];

    colorBotPose[0] = blueBotPose[0];
    colorBotPose[1] = blueBotPose[1];
    colorBotPose[2] = blueBotPose[2];
    colorBotPose[3] = blueBotPose[3];
    colorBotPose[4] = blueBotPose[4];
    colorBotPose[5] = blueBotPose[5];
    colorBotPose[6] = blueBotPose[6];
    colorBotPose[7] = blueBotPose[7];
    colorBotPose[8] = blueBotPose[8];
    colorBotPose[10] = blueBotPose[10];

    if(hasTarget()){
      colorBotPose[9] = blueBotPose[9];
      colorBotPose[11] = getTID();
    }

    return colorBotPose; 
  }

  public long getTID(){
    long tid = this.tid.getInteger(-1);
    
    return tid;
  }

  public void setData(){
    double[] temp = getBlueBotPose();
    Pose2d currentPose = drivebase.getPose();
    double distance = Math.sqrt(Math.pow(temp[0]- currentPose.getX(),2) + Math.pow(temp[1]- currentPose.getY() ,2)); //Calculates distance from Apriltag to robot
    double distancefromAprilTag = 0.8; //0.8
    double distancefromLimeLight = 2.5; //2.5

    if(hasTarget()){
       
        double xyStds= 0;
        double degStds = 0;
        Matrix<N3, N1> stds = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(limelightMeasurement.tagCount >= 2 ){ //Checks if Limelight sees 2 Apriltag
            xyStds = 0.5; 
            degStds = 6 * Math.PI / 180;

            //System.out.println("lime light data");
        
        }
        else if ((temp[9] < distancefromLimeLight) && (distance < distancefromAprilTag)) { //Checks if within distance of apriltag and limelight
            xyStds = 1.0;
            degStds = 12 * Math.PI / 180;

        }
        else{
          return;
        }
          stds.set(0,0,xyStds);
          stds.set(1,0,xyStds);
          stds.set(2,0, degStds * Math.PI / 180);
          Rotation2d degree = new Rotation2d(temp[5]* Math.PI / 180);
          Pose2d newPose = new Pose2d(temp[0],temp[1],degree); //creates new pose2d with limelight data
          
          drivebase.addActualVisionReading(newPose ,Timer.getFPGATimestamp() - (temp[6]/1000.0),stds); //Changes standard dev base on apriltags and drive
  }

  }
}