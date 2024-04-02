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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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
    public static final int ARM_MAX_ANGLE = 330;
    public static final int ARM_MIN_ANGLE = 10;
    public static final int ROTATION_TO_DEGREES = 360;
    public static final double GEAR_RATIO = 118.587767088;
    public static final double ENCODER_RAW_TO_ROTATION = 8132.;
    public static final double ENCODER_OFFSET = 21.8; 
    public static final int ARM_ID = 18;
    public static final int MAX_VOLT = 12;

    //
    public static final double KP = 5.8;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KSVOLTS = 1.5; 
    public static final double KGVOLTS = 0;
    public static final double KVVOLTS = 1.8;
    public static final double KAVOLTS = 0;
    public static final double VEL = 230 * Math.PI / 180;
    public static final double ACC = 360 * Math.PI / 180;
    //
    //Arm ID Jalen Tolbert
    public static final int ENCODER_DIO_SLOT = 0;
    public static final int AMP_ANGLE = 300;
    public static final int TRAP_ANGLE = 270;
    public static final int SPEAKER_LOW_ANGLE = 165;
    public static final int SPEAKER_HIGH_ANGLE = 238;
    public static final int INTAKE_ANGLE = 60;
    public static final int CLIMBING_ANGLE = 45;
  }

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private ArmFeedforward armFeedForward;
  public boolean emergencyStop = false;
  private double prevArmAngle;
  public boolean booster = false;

  public ArmSubsystem(int armId, String armCanbus, int channel1)
  {
    super(new ProfiledPIDController(Arm.KP, Arm.KI, Arm.KD, new Constraints(Arm.VEL, Arm.ACC)));
    getController().setTolerance(2 * (Math.PI/180));
    //TrapezoidProfile either velocity or position
    m_arm = new TalonFX(armId, armCanbus);
    m_encoder = new DutyCycleEncoder(channel1);
    armFeedForward = new ArmFeedforward(Arm.KSVOLTS, Arm.KGVOLTS, Arm.KVVOLTS, Arm.KAVOLTS);
        
    targetArmAngle(encoderGetAngle());
    enable();

    prevArmAngle = encoderGetAngle();

    UtilMotor.configMotor(m_arm, .11, 0, 0, .12, 15, 50, true);      

    Shuffleboard.getTab("Arm").addDouble("Encoder Angle", ()->encoderGetAngle());
    Shuffleboard.getTab("Arm").addDouble("Goal in degrees", ()->getController().getGoal().position * (180/Math.PI));
    
    if(DriverStation.isTest()) {
      Shuffleboard.getTab("Arm").addDouble("Arm Stator Current", () -> m_arm.getStatorCurrent().getValueAsDouble());
      Shuffleboard.getTab("Arm").addDouble("Arm Rotor Velocity", () -> m_arm.getRotorVelocity().getValueAsDouble());
      Shuffleboard.getTab("Arm").addDouble("Arm Temperature", () -> m_arm.getDeviceTemp().getValueAsDouble());
    }

    DataLogManager.log("Arm P: " + Arm.KP);
    DataLogManager.log("Arm I: " + Arm.KI);
    DataLogManager.log("Arm D: " + Arm.KD);
    DataLogManager.log("Arm Velocity: " + Arm.VEL);
    DataLogManager.log("Arm Acceleration: " + Arm.ACC);
    DataLogManager.log("Arm kS: " + Arm.KSVOLTS);
    DataLogManager.log("Arm kG: " + Arm.KGVOLTS);
    DataLogManager.log("Arm kV: " + Arm.KVVOLTS);
    DataLogManager.log("Arm kA: " + Arm.KAVOLTS);
  }

  //Looking at the left of the robot, counterclockwise arm spin is positive
  public void spinArm(double speed)
  {
    if(speed < -Arm.MAX_VOLT) { 
      speed = -Arm.MAX_VOLT;
    }
    else if (speed > Arm.MAX_VOLT) { 
      speed = Arm.MAX_VOLT;
    }

    if(booster)
    {
      speed = Arm.MAX_VOLT;
    }
    
    if(encoderGetAngle() < 0) {
      speed = -5;
    }

    if(encoderGetAngle() >= 300) {
      if (speed > 0.5) {
        speed = 0.5;
      }
    }

    VoltageOut armSpinRequest = new VoltageOut(speed, true, false, false, false);
    m_arm.setControl(armSpinRequest);
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
    m_arm.stopMotor();
 }

 //gets the angle from the encoder(it's *potentially* offset from the motor by: [add value])
  public double encoderGetAngle() {

    return m_encoder.getAbsolutePosition()*Arm.ROTATION_TO_DEGREES - Arm.ENCODER_OFFSET;
  }

  public boolean checkEncoderAngleForClimb() {
    return (encoderGetAngle() >= 125 && encoderGetAngle() <= 270);
  }

  @Override
  public void useOutput(double output, State setPoint)
  {
    double currentArmAngle = encoderGetAngle();
    if (Math.abs(currentArmAngle - prevArmAngle) >= 50) {
      emergencyStop = true;
    }
    prevArmAngle = currentArmAngle;


    double feedForward = armFeedForward.calculate(setPoint.position, setPoint.velocity);
    if(!isEmergencyStop())
    {
      SmartDashboard.putNumber("Arm PID output", output);
      SmartDashboard.putNumber("Arm feed forward", feedForward);
      SmartDashboard.putNumber("Arm speed", output + feedForward);
      SmartDashboard.putNumber("Arm FF setPoint Position", setPoint.position * 180 / Math.PI);
      SmartDashboard.putNumber("Arm FF setPoint velocity", setPoint.velocity * 180 / Math.PI);
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