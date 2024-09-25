// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ReactionIOReal implements ReactionIO{
    private TalonFX m_reactionArm = new TalonFX(ReactionConstants.REACTION_ID, ReactionConstants.REACTION_CAN);
    
    public ReactionIOReal(){
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        

        motorOutput.NeutralMode = NeutralModeValue.Coast;

        currentConfig.withStatorCurrentLimitEnable(true);
        currentConfig.withStatorCurrentLimit(20);
    
        TalonFXConfigurator reactionBarConfigurator = m_reactionArm.getConfigurator();
        reactionBarConfigurator.apply(motorOutput);
        reactionBarConfigurator.apply(currentConfig);
    }

    public double getReactionBarPosition(){
        return m_reactionArm.getPosition().getValueAsDouble();
    }

    public void setReactionBarSpeed(double pidOutput){
        m_reactionArm.set(pidOutput);
    }

    public void update(){

    }
}
