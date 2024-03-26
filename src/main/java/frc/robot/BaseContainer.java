package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public interface BaseContainer 
{
    default String getName()
    {
        return this.getClass().getName();
    }

    default Command getAutonomousCommand()
    {
        return new InstantCommand();
    }

    default String getDrivetrainName() {
        return "none";
    }

    default Command teleopInitReset() {
       return Commands.none();
    }

    default Command autoInitReset() {
       return Commands.none();
    }
}