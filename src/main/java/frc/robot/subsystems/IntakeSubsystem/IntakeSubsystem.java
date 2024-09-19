// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_intakeArm = new TalonFX(IntakeContants.INTAKE_ARM_ID, IntakeContants.INTAKE_ARM_CAN);
  private TalonFX m_intakeSpin = new TalonFX(IntakeContants.INTAKE_SPIN_ID, IntakeContants.INTAKE_SPIN_CAN);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeContants.INTAKE_ENCODER_CHANNEL_ID);
  private DigitalInput noteSensor = new DigitalInput(IntakeContants.INTAKE_NOTE_SENSOR_CHANNEL_ID);

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(IntakeContants.kVel, IntakeContants.kAcc);
  private ProfiledPIDController pidController = new ProfiledPIDController(IntakeContants.kP, IntakeContants.kI, IntakeContants.kD, constraints);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // put commands to shuffleboard for testing
    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout intakeCommandsLayout = tab.getLayout("Intake Commands", BuiltInLayouts.kList)
      .withSize(2,2)
      .withProperties(Map.of("Label position", "HIDDEN"));
    
    intakeCommandsLayout.add(deployIntake());
    intakeCommandsLayout.add(retractIntake());

    SmartDashboard.putData(this);
  }

  public double getIntakeArmAngle() {
    return Units.rotationsToDegrees(encoder.getAbsolutePosition()) - IntakeContants.ENCODER_OFFSET;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pidController.calculate(getIntakeArmAngle());

    pidOutput = MathUtil.clamp(pidOutput, -1, 1);

    m_intakeArm.set(pidOutput);
  }

  public Command deployIntake() {
    return this.runOnce(() -> {
      pidController.setGoal(IntakeContants.DOWN_POSITION);
    })
    .alongWith(Commands.run(() -> {
      m_intakeSpin.set(0.8);
    }))
    .until(() -> noteSensor.get())
    .andThen(retractIntake())
    .withName("Intake: deploy intake");
  }

  public Command retractIntake() {
    return this.runOnce(() -> {
      pidController.setGoal(IntakeConstants.UP_POSITION);
    })
    .andThen(Commands.idle().until(() -> pidController.atGoal()))
    .withName("Intake: retract intake");
  }
}
