// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.hardware.core.CoreTalonFX; //Core?


public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_arm;

  public ArmSubsystem(int armId, String canbus)
  {
    m_arm = new TalonFX(armId, canbus);
  }

  public void setArmAngle(double angle) {
    m_arm.setPosition(angle);
  }

  //MotionMagicVoltage​(double Position, boolean EnableFOC, double FeedForward, int Slot, boolean OverrideBrakeDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)
  public void spinArmMotor (double pos, double feedForward)
  {
    MotionMagicVoltage armSpinRequest = new MotionMagicVoltage(pos, true, feedForward, 0, false, true, false);
    m_arm.setControl(armSpinRequest);
  }

  public void stopArmMotor() {
    m_arm.stopMotor();
 }

  public double getArmPos() {
    return m_arm.getPosition().getValue();
  }

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
