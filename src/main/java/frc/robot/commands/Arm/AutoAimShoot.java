// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.UtilMath;
import frc.robot.commands.driveCommands.BackSpeaker;
import frc.robot.commands.driveCommands.FaceSpeaker;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimShoot extends SequentialCommandGroup {
  /** Creates a new AutoAimShootOverHand. */
  public AutoAimShoot(SwerveSubsystem s, ArmSubsystem a, PizzaBoxSubsystem p, TeleopDrive t, boolean underhand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (underhand) {
      addCommands(
        new FaceSpeaker(t),
        new ScoreInSpeakerAdjustable(p, a, UtilMath.getArmAngle(s.getPose(), underhand)),
        new FaceSpeaker(t)
        );
    } else {
      addCommands(
        new BackSpeaker(t),
        new ScoreInSpeakerAdjustable(p, a, UtilMath.getArmAngle(s.getPose(), underhand)),
        new BackSpeaker(t)
      );
    }
  }
}
