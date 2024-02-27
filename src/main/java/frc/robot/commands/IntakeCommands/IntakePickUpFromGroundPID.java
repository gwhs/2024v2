// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class IntakePickUpFromGroundPID extends PIDCommand {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private static PIDController intakeController = new PIDController(.015, .001, .0);
  
  public IntakePickUpFromGroundPID(IntakeSubsystem intakeSubsystem, int velocity, int accleration) {
    super(intakeController, ()-> intakeSubsystem.encoderGetAngle(), () -> 0,
            (final double speed) -> 
            {intakeSubsystem.spinIntakeArm(-speed);
              intakeSubsystem.spinIntakeMotor(velocity, accleration);}
            , intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    intakeController.setTolerance(Constants.IntakeConstants.TOLERANCE);
    //Shuffleboard.getTab("intake").add(intakeController);
  }

  // public void end(boolean interrupted) {
  //   intakeSubsystem.stopArmMotor();
  // }

  //Returns true when the command should end.
  //@Override
  public boolean isFinished() {
    boolean sensorValue = intakeSubsystem.isNotePresent();
    if(sensorValue) {
      intakeSubsystem.stopArmMotor();
      return sensorValue;
    }
    else {
      return sensorValue;
    }
  }

}
