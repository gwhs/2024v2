// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ReactionIOReal {
    private TalonFX m_reactionArm = new TalonFX(ReactionConstants.REACTION_ID, ReactionConstants.REACTION_CAN);
    
    public StatusSignal<Double> getReactionBarPosition(){
        return m_reactionArm.getPosition();
    }

    public void setReactionBarSpeed(double pidOutput){
        m_reactionArm.set(pidOutput);
    }
}
