// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testcontainers;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.subsystems.ReactionSubsystem;

public class ReactionArmContainer implements BaseContainer
{
    private final ReactionSubsystem m_ArmSubsystem;
    
    CommandXboxController xbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);



    public ReactionArmContainer() {
        m_ArmSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);
        ShuffleboardTab ReactionArm = Shuffleboard.getTab("Reaction Arm");
        ReactionArm.addDouble("Arm Pos", ()-> m_ArmSubsystem.getPos());
        
        
         configureBindings();
     }

     public void configureBindings()
     {
        Shuffleboard.getTab("Reaction Arm").add("Extend", new Extend(m_ArmSubsystem));
        Shuffleboard.getTab("Reaction Arm").add("Retract", new Retract(m_ArmSubsystem));

        xbox.a().onTrue(new Extend(m_ArmSubsystem));
        xbox.b().onFalse(new Retract(m_ArmSubsystem));
        
     }
}
