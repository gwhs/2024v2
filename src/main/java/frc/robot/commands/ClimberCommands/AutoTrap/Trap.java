// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.AutoTrap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ScoreInTrapStutter;
import frc.robot.commands.ClimberCommands.AutoClimb.ClimbDown;
import frc.robot.commands.ClimberCommands.AutoClimb.ClimbUp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Trap extends SequentialCommandGroup {

  //private ArmSubsystem armsubsystem;

  /** Creates a new Trap. */
  
  public Trap(Climbsubsystem c, SwerveSubsystem s, ArmSubsystem a, PizzaBoxSubsystem p, ReactionSubsystem r) {

    addCommands(
          /*new align(), */
          new ClimbUp(c, s, a, r), 
          new ScoreInTrapStutter(p, a),
          new ClimbDown(c, s, a)
    );
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

}
