// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.UtilMath;
import frc.robot.commands.driveCommands.DecreaseSpeed;
import frc.robot.commands.driveCommands.FaceAmp;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aimbot extends SequentialCommandGroup {
  /** Creates a new Aimbot. */
  public Aimbot(TeleopDrive t, ArmSubsystem a, PizzaBoxSubsystem p, SwerveSubsystem s) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(()-> {t.isSlow = true; t.isFaceSpeaker = true;}),
      new ScoreInSpeakerAdjustable(p, a, ()-> UtilMath.overhand.get(UtilMath.distanceFromSpeaker(()-> s.getPose()))),
      Commands.runOnce(()-> {t.isSlow = false; t.isFaceSpeaker = false;})
    );
  }
}
