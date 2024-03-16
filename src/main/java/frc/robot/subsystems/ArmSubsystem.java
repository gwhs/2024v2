// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Util.UtilMotor;


public class ArmSubsystem extends ProfiledPIDSubsystem {

  public static final class Arm {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int ARM_MAX_ANGLE = 335;
    public static final int ARM_MIN_ANGLE = 0;
    public static final int ROTATION_TO_DEGREES = 360;
    public static final double GEAR_RATIO = 118.587767088;
    public static final double ENCODER_RAW_TO_ROTATION = 8132.;
    public static final double ENCODER_OFFSET = 0; 
    public static final int ARM_ID = 18;
    //
    public static final double KSVOLTS = 0; 
    public static final double KGVOLTS = .355;
    //
    //Arm ID Jalen Tolbert
    public static final int ENCODER_DIO_SLOT = 0;
    public static final int AMP_ANGLE = 322;
    public static final int TRAP_ANGLE = 290;
    public static final int SPEAKER_LOW_ANGLE = 165;
    public static final int SPEAKER_HIGH_ANGLE = 238;
    public static final int INTAKE_ANGLE = 60;
    public static final int CLIMBING_ANGLE = 45;
  }

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private ArmFeedforward armFeedForward;
  public boolean emergencyStop = false;

  public ArmSubsystem(int armId, String armCanbus, int channel1)
  {
    super(new ProfiledPIDController(5.35, .25, 0, new Constraints(3.5*Math.PI, 27)));
    getController().setTolerance(2 * (Math.PI/180));
    //TrapezoidProfile either velocity or position
      // m_arm = new TalonFX(armId, armCanbus);
      m_encoder = new DutyCycleEncoder(channel1);
      armFeedForward = new ArmFeedforward(Arm.KSVOLTS, Arm.KGVOLTS, 0, 0);
          
      targetArmAngle(encoderGetAngle());
      enable();
      //m_encoder.reset();

    // UtilMotor.configMotor(m_arm, .11, 0, 0, .12, 15, 50, true);      

    Shuffleboard.getTab("Arm").addDouble("Encoder Angle", ()->encoderGetAngle()).withWidget(BuiltInWidgets.kGraph)
    .withSize(3,3);
    Shuffleboard.getTab("Arm").addDouble("Goal in degrees", ()->getController().getGoal().position * (180/Math.PI));
    
    // Shuffleboard.getTab("Arm").addDouble("Arm Stator Current", () -> m_arm.getStatorCurrent().getValueAsDouble());
    // Shuffleboard.getTab("Arm").addDouble("Arm Rotor Velocity", () -> m_arm.getRotorVelocity().getValueAsDouble());
    // Shuffleboard.getTab("Arm").addDouble("Arm Acceleration", () -> m_arm.getAcceleration().getValueAsDouble());
    // Shuffleboard.getTab("Arm").addDouble("Arm Temperature", () -> m_arm.getDeviceTemp().getValueAsDouble());

    Shuffleboard.getTab("Arm").addDouble("Arm Encoder test get()", () -> (m_encoder.get() * Constants.IntakeConstants.ROTATION_TO_DEGREES));
  }

  //Looking at the left of the robot, counterclockwise arm spin is positive
 public void spinArm(double speed)
 {
  if(speed < -15) { //Will not be less than minimum angle
    speed = -15;
  }
  else if (speed > 15) { // Will not be greater than maximum angle
    speed = 15;
  }
  VoltageOut armSpinRequest = new VoltageOut(-speed, true, false, false, false);
  // m_arm.setControl(armSpinRequest);
 }

 public void targetArmAngle(double angle)
 {
  double calculatedAng = angle ;
  if(calculatedAng  < Arm.ARM_MIN_ANGLE) { //Will not be less than minimum angle
    calculatedAng = Arm.ARM_MIN_ANGLE;
  }
  else if (calculatedAng > Arm.ARM_MAX_ANGLE ) { // Will not be greater than maximum angle
    calculatedAng = Arm.ARM_MAX_ANGLE;
  }

  setGoal(calculatedAng * Math.PI/180);
 }



  //Stops arm motor
  public void stopArmMotor() {
    // m_arm.stopMotor();
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getAbsolutePosition()*Arm.ROTATION_TO_DEGREES - Arm.ENCODER_OFFSET;
  }

  //Resets encoder angle to 0
  public void resetEncoderAngle()
  {
    m_encoder.reset();
  }

  public boolean checkEncoderAngleForClimb() {
    return (encoderGetAngle() >= 125 && encoderGetAngle() <= 270);
  }

  @Override
  public void useOutput(double output, State setPoint)
  {
    //Comment out for testing purposes
    double feedForward = armFeedForward.calculate(setPoint.position, setPoint.velocity);
    if(!isEmergencyStop())
    {
      SmartDashboard.putNumber("Arm PID output", output);
      SmartDashboard.putNumber("Arm feed forward", feedForward);
      SmartDashboard.putNumber("Arm speed", output + feedForward);
      spinArm(output + feedForward);
    }
    else
    {
      spinArm(0);
    }
  }

  public boolean isEmergencyStop()
  {
    return !(m_encoder.isConnected() && !emergencyStop);
  }

  @Override
  public double getMeasurement()
  {
    return encoderGetAngle() * Math.PI/180;
  }
}