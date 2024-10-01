// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO intakeIO;

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(IntakeConstants.kVel, IntakeConstants.kAcc);
  private ProfiledPIDController pidController = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, constraints);

  public final Trigger isDeployed = new Trigger(() -> MathUtil.isNear(IntakeConstants.DOWN_POSITION, pidController.getGoal().position, 1)).debounce(0.5);
  public final Trigger noteTriggered;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if(RobotBase.isSimulation()) {
      intakeIO = new IntakeIOSim();
      NetworkTableInstance.getDefault().getEntry("Intake/Mode").setString("Simulation");
    }
    else {
      intakeIO = new IntakeIOReal();
      NetworkTableInstance.getDefault().getEntry("Intake/Mode").setString("Real");
    }

    pidController.setGoal(IntakeConstants.UP_POSITION);
    pidController.setTolerance(1);

    noteTriggered = new Trigger(() -> intakeIO.getNoteSensor());
    
    // put commands to shuffleboard for testing
    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout intakeCommandsLayout = tab.getLayout("Intake Commands", BuiltInLayouts.kList)
      .withSize(2,2)
      .withProperties(Map.of("Label position", "HIDDEN"));
    
    intakeCommandsLayout.add(deployIntake());
    intakeCommandsLayout.add(retractIntake());

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pidController.calculate(intakeIO.getIntakeArmAngle());

    pidOutput = MathUtil.clamp(pidOutput, -1, 1);

    intakeIO.setArmSpeed(pidOutput);

    intakeIO.update();

    /* Logging */
    NetworkTableInstance.getDefault().getEntry("Intake/PID Output").setNumber(pidOutput);
    NetworkTableInstance.getDefault().getEntry("Intake/PID Goal").setNumber(pidController.getGoal().position);
    NetworkTableInstance.getDefault().getEntry("Intake/PID Profiled Goal").setNumber(pidController.getSetpoint().position);
    NetworkTableInstance.getDefault().getEntry("Intake/Intake Measured Arm Angle").setNumber(intakeIO.getIntakeArmAngle());
    NetworkTableInstance.getDefault().getEntry("Intake/Intake Spin Speed").setNumber(intakeIO.getSpinSpeed());

    NetworkTableInstance.getDefault().getEntry("Intake/Intake Deployed").setBoolean(isDeployed.getAsBoolean());
    NetworkTableInstance.getDefault().getEntry("Intake/Note Sensor").setBoolean(intakeIO.getNoteSensor());
  }

  public Command deployIntake() {
    return this.runOnce(() -> {
      pidController.setGoal(IntakeConstants.DOWN_POSITION);
    })
    .alongWith(Commands.run(() -> {
      intakeIO.setSpinSpeed(0.8);
    }))
    .until(() -> intakeIO.getNoteSensor())
    .andThen(retractIntake())
    .withName("Intake: deploy intake");
  }

  public Command retractIntake() {
    return this.runOnce(() -> {
      pidController.setGoal(IntakeConstants.UP_POSITION);
      intakeIO.setSpinSpeed(0);
    })
    .andThen(Commands.idle().until(() -> pidController.atGoal()))
    .withName("Intake: retract intake");
  }

  public Command intakeNote() {
    return this.run(() -> {
      intakeIO.setSpinSpeed(1);
    });
  }

  public Command stopIntake() {
    return this.runOnce(() -> {
      intakeIO.setSpinSpeed(0);
    });
  }
}
