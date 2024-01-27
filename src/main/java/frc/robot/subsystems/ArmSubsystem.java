// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


  //Bore encoder, Talonfx motor, Magic motion?, 
  //Functions: getCurrentPosition/getCurrentAngle, setTargetPosition, getTorqueVelocity, setVelocity, 
package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.ctre.phoenix6.hardware.core.CoreTalonFX; //Core?
import com.ctre.phoenix6.configs.MotionMagicConfigs;//Maybe needed?
import com.ctre.phoenix6.configs.TalonFXConfiguration; //Maybe needed?
import com.ctre.phoenix6.signals.ControlModeValue; //Maybe needed

//import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
//import org.littletonrobotics.akit.junction;
//TEST
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_arm;
  private final Encoder m_encoder;
  TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  



  //private ShuffleboardTab tab = Shuffleboard.getTab("Encoder");
  //private GenericEntry encoderPosition = tab.add("Encoder Position", 0).getEntry();
  //private GenericEntry encoderRate =
      // tab.add("Encoder Rate", 0)
      //     .withWidget(BuiltInWidgets.kDial)
      //     .withProperties(Map.of("min", -500, "max", 500))
      //     .getEntry();

  public ArmSubsystem(int armId, String canbus, int channel1, int channel2)
  {
      m_arm = new TalonFX(armId, canbus);
      m_encoder = new Encoder(channel1, channel2, false, Counter.EncodingType.k4X);
  
      m_encoder.reset();
      m_encoder.setSamplesToAverage(5);
      m_encoder.setDistancePerPulse(1. / 256.);
      m_encoder.setMinRate(1.0);
    
  }

  /*
    
    public void setArmAngle(double angle) {
         m_arm.setPosition(angle);
        }
       
     //MotionMagicVoltage​(double Position, boolean EnableFOC, double FeedForward, int Slot, boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
   public void spinArmMotor (double pos)
   {
      MotionMagicVoltage armSpinRequest = new MotionMagicVoltage(pos, true, 0, 0, false, true, false);
      m_arm.setControl(armSpinRequest.withPosition(pos));

    //m_arm.setControl(armSpinRequest);
  }
    
  
   */
  //vel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10
  //accel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10
  public void setAngle(double angle, double vel, double accel) {
    //Pos will be based on motor 
    //Make sure angle is between 0 and 270 degrees! getPosition()
    double adjustedAngle = ((angle - encoderGetAngle() + m_arm.getPosition().getValue())) * Constants.Arm.GEAR_RATIO / Constants.Arm.FALCON_TICKS; //multiply by gearbox and divide by ticks? (times 64 divided by 2048?)
    MotionMagicVoltage m_smoothArmMovement = new MotionMagicVoltage(adjustedAngle, true, 0, 0, false, true, false);

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = vel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10;
    motionMagicConfigs.MotionMagicAcceleration = accel / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10; 

    m_arm.getConfigurator().apply(talonFXConfigs, 0.03);

    m_arm.setControl(m_smoothArmMovement);

    // m_arm.set(
    //     ControlMode.MotionMagic,
    //     angle / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO);
    //m_motor.setControl(m_motmag.withPosition(200));


  }
  public void resetPosition() {
    setAngle(0.0, 0.0, 0.0); 
  }

  public void stopArmMotor() {
    m_arm.stopMotor();
 }

  public double encoderGetAngle() {

    return m_encoder.getRaw()/8132 * 360;
  }

  public void encoderReset() {
    m_encoder.reset();
  }

  public boolean encoderGetStopped() {
    return m_encoder.getStopped();
  }

  // public double getAng() {
  //   return m_arm.getPosition();
  // }
 
  //
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ticks = m_encoder.get();

    double rawAngle = (-m_encoder.getRaw() / 8192. * 360.);
    Logger.getInstance().recordOutput("/Angle", rawAngle);

    //  SmartDashboard.putNumber("Encoder ticks", ticks);
    //  SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    //  SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
    //  encoderPosition.setDouble(ticks);
    //  encoderRate.setDouble(m_encoder.getRate());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
