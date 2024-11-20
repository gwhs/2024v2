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

public class S1_A1_A2 extends PathPlannerAuto {
  public S1_A1_A2(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
      PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(() -> {
    }));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath S1_A1 = PathPlannerPath.fromPathFile("S1-A1");
      PathPlannerPath A1_A2 = PathPlannerPath.fromPathFile("A1-A2");
      PathPlannerPath A1_S1 = PathPlannerPath.fromPathFile("A1-S1");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose = new Pose2d(
          S1_A1.getPoint(0).position,
          S1_A1.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning().onTrue(
          Commands.sequence(
              AutoBuilder.resetOdom(startingPose),
              robotContainer.scoreSpeaker(160),
              AutoBuilder.followPath(S1_A1).alongWith(robotContainer.deployIntake()))
              .withName("S1; Score Preload; S1->A1"));

      /* TODO: At A1/Score at A1, then go to A2 */
      
      /* event("atA1").onTrue(
         Commands.sequence(
          robotContainer.retractIntakePassToPB(),
          robotContainer.scoreSpeaker(0),
          AutoBuilder.followPath(A1_A2)
            .withName("A1, score note, A1->A2")
         )); */
      
      //TODO: atA2, score at A2

     /*  event("atA2").onTrue(
        Commands.sequence(
          robotContainer.retractIntakePassToPB(),
          robotContainer.scoreSpeaker(0)
            .withName("At A2, score note")
        )
      ); */
      event("atA1").and(intakeSubsystem.noteTriggered).onTrue(
        Commands.sequence(
          robotContainer.retractIntakePassToPB()),
          robotContainer.scoreSpeaker(160),
          AutoBuilder.followPath(A1_A2)
          
          );

        
      

      event("atA1").and(intakeSubsystem.noteTriggered.negate()).onTrue(
        Commands.sequence(
          AutoBuilder.followPath(A1_A2)


        )
      );

      event("atA2").and(intakeSubsystem.noteTriggered).onTrue(
        Commands.sequence(
          robotContainer.scoreSpeaker(160)
        
        )
      );

      event("atA2").and(intakeSubsystem.noteTriggered.negate()).onTrue(
        Commands.sequence(
          robotContainer.retractIntake()
        )
      );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
