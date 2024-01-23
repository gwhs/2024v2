// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledcommands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ChangeLEDToGreen extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final LEDSubsystem ledSubsystem;

  public ChangeLEDToGreen(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("green leds have succesfully executed");
    ledSubsystem.setColor(0, 255, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ledSubsystem.getColor(1).equals(new Color(0, 255, 0));
  }
}
