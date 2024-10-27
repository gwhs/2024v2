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

public class S3Leave extends PathPlannerAuto {
  public S3Leave(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(()->{}));

    /* All your code should go inside this try-catch block */
    try {
      /* Load all paths needed */
      PathPlannerPath S3C5 = PathPlannerPath.fromPathFile("S3-C5");

      /* Get starting position of starting path */
      Pose2d startingPose = new Pose2d(
        S3C5.getPoint(0).position, 
        S3C5.getIdealStartingState().rotation());

      /* When autonomous begins: 
      1) Reset pose estimator so robot knows it is at starting position
      2) score pre-load note
      3) follow path */
      isRunning().onTrue(
        Commands.sequence(
          AutoBuilder.resetOdom(startingPose),
          robotContainer.scoreSpeaker(160),
          AutoBuilder.followPath(S3C5)
          )
          .withName("S3; Score Prelod; S3->C5"));
    } 
    catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
