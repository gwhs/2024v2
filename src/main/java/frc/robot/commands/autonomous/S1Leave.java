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

public class S1Leave extends SequentialCommandGroup {
  public S1Leave(
      GameRobotContainer robotContainer,
      ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem,
      PizzaBoxSubsystem pizzaBoxSubsystem) {
    setName("S1-Leave");
    addRequirements(armSubsystem, intakeSubsystem, pizzaBoxSubsystem);

    /* All your code should go inside this try-catch block */
    try {
      /* Load all paths needed */
      // TODO: Load your path
      

      addCommands(
          // TODO: Step 1: Reset where the robot think it is: at the starting position S3
          

          // TODO: Step 2: Score pre-load Note
          

          // TODO: Step 3: Leave Starting Zone
          
      );
    } 
    catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }

  }
}
