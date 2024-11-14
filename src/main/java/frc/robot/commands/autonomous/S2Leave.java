// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameRobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;

public class S2Leave extends PathPlannerAuto {
  public S2Leave(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath S2A2 = PathPlannerPath.fromPathFile("S2-A2");
      PathPlannerPath A2A3 = PathPlannerPath.fromPathFile("A2-A3");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose = new Pose2d(
        S2A2.getPoint(0).position, 
        S2A2.getIdealStartingState().rotation());

      Pose2d nextPose = new Pose2d(
        A2A3.getPoint(0).position, 
        A2A3.getIdealStartingState().rotation());        

      new EventTrigger("wait10Seconds").whileTrue(Commands.waitSeconds(10));  
      /* TODO: Autonomous Routine's first actions */
      isRunning().onTrue(
        Commands.sequence(
          // TODO: Reset robot odometry with starting position so robot knows where it is
          AutoBuilder.resetOdom(startingPose),
          // TODO: Score pre-load note to speaker
          robotContainer.scoreSpeaker(236),
          // TODO: Follow Path
          AutoBuilder.followPath(S2A2).alongWith(robotContainer.deployIntake())
          
          // TODO: Name of command
          .withName("S2A2; Score Preload;")));

      event("atA2").onTrue(
        Commands.sequence(
          robotContainer.retractIntakePassToPB(),
          robotContainer.scoreSpeaker(236)
          .withName("at A2 with note"),
          AutoBuilder.resetOdom(nextPose),         
          AutoBuilder.followPath(A2A3).alongWith(robotContainer.deployIntake())
          ));
          
      event("atA3").onTrue(
        Commands.sequence(
          robotContainer.retractIntake(),
          robotContainer.scoreSpeaker(236)
          .withName("at A3 with note")
        ));
    } 

    catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
