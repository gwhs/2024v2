// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Arm.ResetArm;
import frc.robot.commands.IntakeCommands.IntakeResetArm;
import frc.robot.commands.LimelightCommands.toggleLimelightPoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.LimelightHelpers.LimelightHelpers;
import frc.robot.subsystems.PizzaBoxSubsystem.PizzaBox;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetupShuffleboard extends SubsystemBase {
  /** Creates a new SetupShuffleboard. */
  // private static UsbCamera usbCamera = new UsbCamera("USB Camera", 1);
  // private static MjpegServer mjpegServer = new MjpegServer("Serve_USB CAMERA", 1181);

  public SetupShuffleboard() {
  }

  public static void setupShuffleboard(SwerveSubsystem swerve, PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, LimeLightSub limelightSubsystem, SendableChooser<Command> chooser){
    //LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    // Shuffleboard.getTab("GameTab").add("Camera",usbCamera);
    // Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5800").withSize(4,3).withPosition(5, 0);
    
    Shuffleboard.getTab("GameTab").add("Filed", swerve.getField2d()).withSize(4, 3).withPosition(0, 0);
    Shuffleboard.getTab("GameTab").add("Autonomous Chooser", chooser).withSize(2, 1).withPosition(0, 0);

    Shuffleboard.getTab("GameTab").addBoolean("Note In Pizza Box", ()-> pizzaBoxSubsystem.hasNote);
    Shuffleboard.getTab("GameTab").addBoolean("Arm Running", ()-> armSubsystem.isEmergencyStop())
        .withSize(1, 1).withPosition(4, 3);

    Shuffleboard.getTab("GameTab").addBoolean("Intake Running", ()-> intakeSubsystem.isEmergencyStop())
        .withSize(1, 1).withPosition(5, 3);

    Shuffleboard.getTab("GameTab").add("Reset Arm", new ResetArm(armSubsystem, pizzaBoxSubsystem) )
        .withSize(1,1).withPosition(4,4);
    Shuffleboard.getTab("GameTab").add("Reset Intake", new IntakeResetArm(intakeSubsystem))
        .withSize(1,1).withPosition(5,4);

    Shuffleboard.getTab("GameTab").add("Disable Pose Estimator", new toggleLimelightPoseEstimation(limelightSubsystem))
        .withSize(1,1).withPosition(2,1);

        ShuffleboardTab driveTrainShuffleboardTab = Shuffleboard.getTab("Drive Train");
    
        driveTrainShuffleboardTab.addDouble("X Position", ()->swerve.getPose().getX())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(3,3)
          .withPosition(0, 0);
        driveTrainShuffleboardTab.addDouble("Y Position", ()->swerve.getPose().getY())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(3,3)
          .withPosition(3, 0);
        driveTrainShuffleboardTab.addDouble("Angel", ()->swerve.getPose().getRotation().getDegrees())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(3,3)
          .withPosition(6, 0);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
