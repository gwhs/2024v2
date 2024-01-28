package frc.robot.commands.ClimberCommands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbsubsystem;

public class Climb extends Command {
  /** Creates a new ClimvberCommand. */
  private Climbsubsystem climbersubsystem;
  private double targetPositionTicks;
  private boolean goingUp;
  private double speed;

  //private final ShuffleboardTab tab;


  public Climb(Climbsubsystem climbersubsystem, double inches, double speed) {  
    this.climbersubsystem = climbersubsystem;
    this.targetPositionTicks = climbersubsystem.inchesToTicks(inches);
    this.speed = speed;
    addRequirements(this.climbersubsystem);


    
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climbersubsystem.getPositionLeft() > targetPositionTicks) { // || climbersubsystem.getPositionRight() > targetPositionTicks){
      goingUp = false;
      climbersubsystem.setSpeed(-speed);
    } else {
      goingUp = true;
      climbersubsystem.setSpeed(speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("RightPosition", climbersubsystem.getPosition());
  final ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    ShuffleboardLayout climb =
        tab.getLayout("Climb Distance", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);

    climb.addNumber("Distance", () -> climbersubsystem.ticksToInches(climbersubsystem.getPositionLeft()));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbersubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //CHANGE THIS -- CHANGE THE DISTANCE IT TRAVELS
    if (goingUp){ //if position is greater than where we want to go it stop climbing, and sets to brake mode.
      return (climbersubsystem.getPositionLeft() > targetPositionTicks || climbersubsystem.getPositionLeft() > climbersubsystem.inchesToTicks(29));
             // || (climbersubsystem.getPositionRight() > targetPositionTicks || climbersubsystem.getPositionRight() > climbersubsystem.inchesToTicks(29)); // change the inches
    } else {
      return climbersubsystem.getPositionLeft() < targetPositionTicks || climbersubsystem.getPositionLeft() < -20000; //change the big number
    }
  }
}