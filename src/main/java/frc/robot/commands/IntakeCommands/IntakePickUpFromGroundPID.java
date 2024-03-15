// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class IntakePickUpFromGroundPID extends PIDCommand {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private static PIDController intakeController = new PIDController(.025, .001, .0);
  private double velocity;
  private double accleration;
  
  public IntakePickUpFromGroundPID(IntakeSubsystem intakeSubsystem, double velocity, double accleration) {
    super(intakeController, ()-> intakeSubsystem.encoderGetAngle(), () -> 0,
            (final double speed) -> 
            {
              intakeSubsystem.spinIntakeArm(-speed);
            }
            , intakeSubsystem);
    intakeController.setTolerance(Constants.IntakeConstants.TOLERANCE);

    this.intakeSubsystem = intakeSubsystem;
    this.velocity = velocity;
    this.accleration = accleration;
  }

  @Override
  public void initialize() {
    intakeSubsystem.spinIntakeMotor(velocity, accleration);
  }

  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean sensorValue = intakeSubsystem.isNotePresent();
    if(sensorValue) {
      //intakeSubsystem.stopIntakeMotors();
      intakeSubsystem.spinIntakeMotor(0, 0);
      return true ;
    }
    else {
      return false;
    }
  }

}
