// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class SpinIntakePID extends PIDCommand {
    private double angle;
    static final PIDController intakeController = new PIDController(.005, .005, .0);


  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;

  public SpinIntakePID(IntakeSubsystem intakeSubsystem, final double targetAngle) {
    super(intakeController, ()-> intakeSubsystem.encoderGetAngle(), () -> targetAngle,
            (final double speed) -> 
            {intakeSubsystem.spinIntakeArm(-speed);
            System.out.println(speed);}
            , intakeSubsystem);
    angle = targetAngle;
    this.intakeSubsystem = intakeSubsystem;

  }
//
//  Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     double encoderAng = armSubsystem.encoderGetAngle();
//     return Math.abs(angle - encoderAng) < 1; 
//   }
}