// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;


/** Add your docs here. */
public interface ReactionIO {
    public double getReactionBarPosition();
    public void setReactionBarSpeed(double pidOutput);
    public void update();
}