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

public class S2Leave extends PathPlannerAuto {
  public S2Leave(GameRobotContainer robotContainer, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, PizzaBoxSubsystem pizzaBoxSubsystem) {
    super(Commands.run(()->{}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath S3Leave = PathPlannerPath.fromPathFile("S3-C5");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose = new Pose2d(
        S3Leave.getPoint(0).position, 
        S3Leave.getIdealStartingState().rotation());

      /* TODO: Autonomous Routine's first actions */
      isRunning().onTrue(
        Commands.sequence(
          // TODO: Reset robot odometry with starting position so robot knows where it is

          // TODO: Score pre-load note to speaker

          // TODO: Follow Path
          )
          // TODO: Name of command
          .withName(""));
    } 
    catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
