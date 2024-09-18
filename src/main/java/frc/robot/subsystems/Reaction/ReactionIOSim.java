// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Reaction;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ReactionIOSim {
    private DCMotorSim reactionMotorSim = 
        new DCMotorSim(
            DCMotor.getFalcon500Foc(1), 
            1, 
            0.5);


    public double getReactionBarPosition(){
        return reactionMotorSim.getAngularPositionRad();
    }

    public void setReactionBarSpeed(double speed){
        reactionMotorSim.setInputVoltage(speed * RobotController.getBatteryVoltage());
    }

    public void update(){
        reactionMotorSim.update(0.2);
    }

}
