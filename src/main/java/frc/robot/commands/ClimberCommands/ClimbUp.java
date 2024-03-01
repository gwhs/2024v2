// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbUp extends SequentialCommandGroup {

  /** Creates a new ClimbUp.*/
  
  public ClimbUp(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, ReactionSubsystem r) {
    addCommands(
          // new ParallelCommandGroup(new SpinToArmAngle(a, 135),
          //     new SequentialCommandGroup(new WaitCommand(5), new MotorUp(c, s) /*new Extend(r)*/)),
          // //new driveforward(),
          new ParallelCommandGroup(new SpinToArmAngle(a, 135), new Extend(r)),
          new MotorUp(c, s),
          new MotorDown(c, s)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.

  }
}
