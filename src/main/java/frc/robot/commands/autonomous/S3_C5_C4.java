// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameRobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;

public class S3_C5_C4 extends PathPlannerAuto {
  public S3_C5_C4(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
      PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(() -> {
    }));

    try {
      PathPlannerPath S3C5 = PathPlannerPath.fromPathFile("S3-C5");
      PathPlannerPath C5S3 = PathPlannerPath.fromPathFile("C5-S3");
      PathPlannerPath S3C4 = PathPlannerPath.fromPathFile("S3-C4");
      PathPlannerPath C4S3 = PathPlannerPath.fromPathFile("C4-S3");
      PathPlannerPath S3Center = PathPlannerPath.fromPathFile("S3-Center");
      PathPlannerPath C5C4 = PathPlannerPath.fromPathFile("C5-C4");

      Pose2d startingPose = new Pose2d(
          S3C5.getPoint(0).position,
          S3C5.getIdealStartingState().rotation());

      isRunning().onTrue(
          Commands.sequence(
              AutoBuilder.resetOdom(startingPose),
              robotContainer.scoreSpeaker(160),
              AutoBuilder.followPath(S3C5).alongWith(robotContainer.deployIntake())));

      /* Branch: Robot at C5 and successfully intake note*/
      event("atC5").and(intakeSubsystem.noteTriggered).onTrue(
        Commands.sequence(
          AutoBuilder.followPath(C5S3).alongWith(robotContainer.retractIntakePassToPB()),
          robotContainer.scoreSpeaker(160),
          AutoBuilder.followPath(S3C4).alongWith(robotContainer.deployIntake())
        )
      );

      /* Branch: Robot at C5 and failed to intake note*/
      event("atC5").and(intakeSubsystem.noteTriggered.negate()).onTrue(
        Commands.sequence(
          AutoBuilder.followPath(C5C4)
        )
      );

      /* Branch: Robot at C4 and successfully intake note*/
      event("atC4").and(intakeSubsystem.noteTriggered).onTrue(
        Commands.sequence(
          AutoBuilder.followPath(C4S3).alongWith(robotContainer.retractIntakePassToPB()),
          robotContainer.scoreSpeaker(160),
          AutoBuilder.followPath(S3Center)
        )
      );

      /* Branch: Robot at C4 and failed to intake note*/
      event("atC4").and(intakeSubsystem.noteTriggered.negate()).onTrue(
        Commands.sequence(
          robotContainer.retractIntake()
        )
      );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
