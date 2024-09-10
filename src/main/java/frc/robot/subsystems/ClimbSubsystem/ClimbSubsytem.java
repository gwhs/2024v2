// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbSubsystem;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class ClimbSubsytem extends SubsystemBase {
    private TalonFX m_leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private TalonFX m_rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
    private Constraints constraints = new Constraints(ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCELERATION);


    private ProfiledPIDController leftpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP, ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);
    private ProfiledPIDController rightpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP, ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);

    public ClimbSubsytem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Testing");
        ShuffleboardLayout climbCommandsLayout = tab.getLayout("TestingCommands", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "HIDDEN"));

        climbCommandsLayout.add(motorUp());
        climbCommandsLayout.add(motorDown());

}

    @Override
    public void periodic() {
        double rightpidOutput = rightpidController.calculate(m_rightClimbMotor.getPosition().getValue());
        double leftpidOutput = leftpidController.calculate(m_leftClimbMotor.getPosition().getValue());

        m_leftClimbMotor.set(leftpidOutput);
        m_rightClimbMotor.set(rightpidOutput);
    }

    public Command motorUp() {
        return this.runOnce(()   -> {
            leftpidController.setGoal(ClimbConstants.LEFT_UP_POSITION);
            rightpidController.setGoal(ClimbConstants.RIGHT_UP_POSITION);
        });
    }

    public Command motorDown() {
        return this.runOnce(()   -> {
            leftpidController.setGoal(ClimbConstants.LEFT_DOWN_POSITION);
            rightpidController.setGoal(ClimbConstants.RIGHT_DOWN_POSITION);
        });
    }
}