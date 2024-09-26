// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ReactionSubsystem extends SubsystemBase {
  private ReactionIO reactionIO;
  private PIDController pidController = new PIDController(ReactionConstants.kP, ReactionConstants.kI,
      ReactionConstants.kD);

  /** Creates a new ReactionSubsystem. */
  public ReactionSubsystem() {
    reactionIO = new ReactionIOReal();
    if (RobotBase.isSimulation()) {
      reactionIO = new ReactionIOSim();
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout reactionCommandsLayout = tab.getLayout("Reaction Commands", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label Position", "HIDDEN"));
    reactionCommandsLayout.add(extendReactionBar());
    reactionCommandsLayout.add(retractReactionBar());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double reactionBarArmPosition = reactionIO.getReactionBarPosition();
    double pidOutput = pidController.calculate(reactionBarArmPosition);

    pidOutput = MathUtil.clamp(pidOutput, -1, 1);

    reactionIO.setReactionBarSpeed(pidOutput);

    reactionIO.update();

    NetworkTableInstance.getDefault().getEntry("Reaction Bar/P").setNumber(ReactionConstants.kP);
    NetworkTableInstance.getDefault().getEntry("Reaction Bar/I").setNumber(ReactionConstants.kI);
    NetworkTableInstance.getDefault().getEntry("Reaction Bar/D").setNumber(ReactionConstants.kD);
    NetworkTableInstance.getDefault().getEntry("Reaction Bar/PID Controller Goal")
        .setNumber(pidController.getSetpoint());
    NetworkTableInstance.getDefault().getEntry("Reaction Bar/Current Pos")
        .setNumber(reactionBarArmPosition);
    NetworkTableInstance.getDefault().getEntry("Reaction Bar/Current PID Val")
        .setNumber(pidOutput);
  }

  public Command extendReactionBar() {
    return this.runOnce(() -> {
      pidController.setSetpoint(ReactionConstants.EXTENDED_POSITION);
    }).withName("Extend Reaction Bar");
  }

  public Command retractReactionBar() {
    return this.runOnce(() -> {
      pidController.setSetpoint(ReactionConstants.RETRACTED_POSITION);
    }).withName("Retract Reaction Bar");
  }

}
