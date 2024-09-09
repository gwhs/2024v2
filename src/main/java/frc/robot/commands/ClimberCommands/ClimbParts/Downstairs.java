// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands.ClimbParts;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGroundPID;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReactionSubsystem;

public class Downstairs extends SequentialCommandGroup {
  /** Creates a new PrepClimb. */
  public Downstairs(ArmSubsystem a, ReactionSubsystem r, IntakeSubsystem i) {
    addCommands(
      new SpinToArmAngle(a,  ArmSubsystem.Arm.ARM_MAX_ANGLE)
      
      .alongWith((new IntakePickUpFromGroundPID(i, 0.6, 0.0)))
      .alongWith((new Extend(r)))




      // Commands.runOnce(()->DataLogManager.log("Command Start: PrepClimb")),
      // new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ARNGLE_FOR_SERVO).withTimeout(4),
      // new SwingForwardServoTheSecond(p),
      // new WaitCommand(0.7),
      // new SpinToArmAngle(a, Constants.ClimbConstants.CLIMB_ARM_ANGLE).withTimeout(1),
      // Commands.waitUntil(()->a.checkEncoderAngleForClimb()), 
      // new MotorUp(c, a).withTimeout(5),
      // Commands.runOnce(()->DataLogManager.log("Command End: PrepClimb"))
    );
  }
}