// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testcontainers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.Robot;
import java.io.File;

import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class ReactionArmContainer implements BaseContainer
{
    private final ReactionSubsystem m_ArmSubsystem;
    
    CommandXboxController xbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);


    public ReactionArmContainer() {
        m_ArmSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);
         configureBindings();
     }

     public void configureBindings()
     {
        xbox.a().onTrue(new Extend(m_ArmSubsystem));
        xbox.b().onFalse(new Retract(m_ArmSubsystem));
        
     }
}
