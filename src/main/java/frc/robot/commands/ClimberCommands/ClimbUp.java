// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climbsubsystem;

public class ClimbUp extends Command {
  /** Creates a new ClimbUp. */
  private Climbsubsystem climbersubsystem;
  private double targetPositionTicks;

  public ClimbUp(Climbsubsystem c) {

    climbersubsystem = c;
    targetPositionTicks = c.inchesToTicks(ClimbConstants.CLIMB_DISTANCE);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbersubsystem.setSpeed(ClimbConstants.CLIMB_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // final ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    // ShuffleboardLayout climb =
    //     tab.getLayout("Climb Distance", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);

    // climb.addNumber("Distance", () -> climbersubsystem.ticksToInches(climbersubsystem.getPositionLeft()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbersubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//(climbersubsystem.getPositionLeft() > targetPositionTicks || climbersubsystem.getPositionLeft() > climbersubsystem.inchesToTicks(29)); //change the inches
  }
}
