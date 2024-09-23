// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ReactionIOReal implements ReactionIO{
    private TalonFX m_reactionArm = new TalonFX(ReactionConstants.REACTION_ID, ReactionConstants.REACTION_CAN);
    
    public double getReactionBarPosition(){
        return m_reactionArm.getPosition().getValueAsDouble();
    }

    public void setReactionBarSpeed(double pidOutput){
        m_reactionArm.set(pidOutput);
    }

    public void update(){

    }
}
