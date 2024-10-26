// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GameRobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;

public class S3Leave extends SequentialCommandGroup {
  public S3Leave(
      GameRobotContainer robotContainer,
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem,
      PizzaBoxSubsystem pizzaBoxSubsystem) {
    setName("S3-Leave");
    addRequirements(armSubsystem, intakeSubsystem, pizzaBoxSubsystem);

    /* All your code should go inside this try-catch block */
    try {
      /* Load all paths needed */
      PathPlannerPath S3Leave = PathPlannerPath.fromPathFile("S3-Leave");

      addCommands(
          // Step 1: Reset where the robot think it is: at the starting position S3
          AutoBuilder.resetOdom(S3Leave.getStartingDifferentialPose()),

          // Step 2: Score pre-load Note
          robotContainer.scoreSpeaker(160),

          // Step 3: Leave Starting Zone
          AutoBuilder.followPath(S3Leave)
      );
    } 
    catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }

  }
}
