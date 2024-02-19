// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class SpinIntakePID extends PIDCommand {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;

  public SpinIntakePID(PIDController intakeController, IntakeSubsystem intakeSubsystem, final double targetAngle) {
    super(intakeController, ()-> intakeSubsystem.encoderGetAngle(), () -> targetAngle,
            (final double speed) -> 
            {intakeSubsystem.spinIntakeArm(-speed);
            System.out.println(speed);}
            , intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  //Returns true when the command should end.
  //@Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

}