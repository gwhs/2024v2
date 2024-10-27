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

public class S3_C5 extends PathPlannerAuto {
  public S3_C5(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
      PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(() -> {
    }));

    /* All your code should go inside this try-catch block */
    try {
      /* Load all paths needed */
      PathPlannerPath S3C5 = PathPlannerPath.fromPathFile("S3-C5");
      PathPlannerPath C5S3 = PathPlannerPath.fromPathFile("C5-S3");

      /* Get starting position of starting path */
      Pose2d startingPose = new Pose2d(
          S3C5.getPoint(0).position,
          S3C5.getIdealStartingState().rotation());

      /*
       * When autonomous begins:
       * 1) Reset pose estimator so robot knows it is at starting position
       * 2) score pre-load note
       * 3) follow path
       */
      isRunning().onTrue(
          Commands.sequence(
              AutoBuilder.resetOdom(startingPose),
              robotContainer.scoreSpeaker(160),
              AutoBuilder.followPath(S3C5))
              .withName("S3; Score Preload; S3->C5"));

      /*
       * Deploy intake when event marker "deployIntake" is triggered
       */
      event("deployIntake").onTrue(robotContainer.deployIntake());

      /*
       * Branching off at C5 note depending if intake successfully intake a note
       */
      // Robot is at C5 and successfull intaked note -> go back to speaker and score
      event("atC5").and(intakeSubsystem.noteTriggered).onTrue(
          Commands.sequence(
              AutoBuilder.followPath(C5S3).alongWith(robotContainer.retractIntakePassToPB()),
              robotContainer.scoreSpeaker(160),
              AutoBuilder.followPath(S3C5))
              .withName("At C5 with Note; C5->Score")
              .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

      // Robot is at C5 and failed to intake note -> stay at C5 and retract intake
      event("atC5").and(intakeSubsystem.noteTriggered.negate()).onTrue(
          Commands.sequence(
              robotContainer.retractIntake())
              .withName("At C5 without Note; Stay"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
