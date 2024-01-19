package frc.robot.subsystems.ArmSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
//

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

//import org.littletonrobotics.junction.AutoLog;

public class Arm extends SubsystemBase {
  public int modeNum = -1;
  private TalonFX armMotor;

  /** Creates a new MagicMotion. */
  public Arm(int id, String can) {
    armMotor = new TalonFX(0, "testCanivore");
    resetPosition();
  }

  @Override
  public void periodic() {
 //   updateInputs(inputs);
    // This method will be called once per scheduler run
  }

  public void robotInit() {
    /* Factory default hardware to prevent unexpected behavior */
    // armMotor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
//    armMotor.configSelectedFeedbackSensor(
//        TalonFXFeedbackDevice.IntegratedSensor,
//        Constants.Arm.kPIDLoopIdx,
//        Constants.Arm.kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %).
    The default deadband is 0.04 (4 %) */
//    armMotor.configNeutralDeadband(0.001, Constants.Arm.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to have green LEDs
     * when driving Talon Forward / Requesting Postiive Output Phase sensor to have positive
     * increment when driving Talon Forward (Green LED)
     */

    /* Set acceleration and vcruise velocity - see documentation */

    /* Zero the sensor once on robot boot up */
    //m_fx.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));

    // MotionMagicVelocityTorqueCurrentFOC​(double Velocity, double Acceleration, boolean EnableFOC,
    // double FeedForward, int Slot, boolean OverrideCoastDurNeutral, boolean LimitForwardMotion, boolean LimitReverseMotion)	
    MotionMagicVelocityTorqueCurrentFOC armVelocity = new MotionMagicVelocityTorqueCurrentFOC(0, 0, false, 0, 0, false, false, false);
    armMotor.setSelectedSensorPosition(0, Constants.Arm.kPIDLoopIdx, Constants.Arm.kTimeoutMs);
  }

  public void setAng(double angle, double vel, double accel) {
    // double velocity = vel / 360 * Constants.FALCON_TICKS * 20;
    // double acceleration = accel / 360 * Constants.FALCON_TICKS * 20;

    // System.out.println("Velocity: " + velocity);
    // System.out.println("Acceleration: " + acceleration);

    armMotor.configMotionCruiseVelocity(
        vel / 360. * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10,
        Constants.Arm.kTimeoutMs);
    armMotor.configMotionAcceleration(
        accel / 360. * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO * 10,
        Constants.Arm.kTimeoutMs);

    armMotor.set(
        ControlMode.MotionMagic,
        angle / 360 * Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO);
  }

  public void resetPosition() {
    armMotor.setSelectedSensorPosition(0);
  }

  public double getAng() {
    return armMotor.getSelectedSensorPosition();
  }

  public double getAngDegrees() {
    return getAng() / (Constants.Arm.FALCON_TICKS * Constants.Arm.GEAR_RATIO) * 360;
  }

  public void enableBrakeMode(boolean brake) {
    if (brake) {
      armMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      armMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public int getMode() {
    return this.modeNum;
  }

  public boolean isConeMode() {
    return this.modeNum == 1;
  }

  public void swapMode() {
    if (this.modeNum == 1) {
      this.modeNum = -1;
      System.out.print(this.modeNum);
    } else {
      this.modeNum = 1;
      System.out.print(this.modeNum);
    }
  }

  public void neutralOutput() {
    //armMotor.neutralOutput();
  }
}