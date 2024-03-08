// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Arm.ResetArm;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.commands.ClimberCommands.ClimbParts.ClimbAndShoot;
import frc.robot.commands.ClimberCommands.ClimbParts.PrepClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.StopClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.UnClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.UnClimbPartTwoThatWillBringDownTheMotor;
import frc.robot.commands.IntakeCommands.IntakeRejectNote;
import frc.robot.commands.IntakeCommands.IntakeResetArm;
import frc.robot.commands.LimelightCommands.toggleCameraMode;
import frc.robot.commands.LimelightCommands.toggleLimelightPoseEstimation;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetupShuffleboard extends SubsystemBase {
  /** Creates a new SetupShuffleboard. */
  // private static UsbCamera usbCamera = new UsbCamera("USB Camera", 1);
  // private static MjpegServer mjpegServer = new MjpegServer("Serve_USB CAMERA", 1181);
  

  public SetupShuffleboard() {
  }

  public static void setupShuffleboard(SwerveSubsystem swerve, PizzaBoxSubsystem pizzaBoxSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, LimeLightSub limelightSubsystem, Climbsubsystem climbSubsystem, ReactionSubsystem reactionSubsystem, SendableChooser<Command> chooser){
    // LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    // Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5800").withSize(4,3).withPosition(5, 0);
    
    Shuffleboard.getTab("GameTab").add("Field", swerve.getField2d())
      .withSize(3, 2)
      .withPosition(0, 0);
    Shuffleboard.getTab("GameTab").add("Autonomous Chooser", chooser)
      .withSize(2, 1)
      .withPosition(0, 2);
    Shuffleboard.getTab("GameTab").add("Disable Pose Estimator", new toggleLimelightPoseEstimation(limelightSubsystem))
      .withSize(1,1)
      .withPosition(2,2);
    Shuffleboard.getTab("GameTab").add("Toggle Camera Mode", new toggleCameraMode(limelightSubsystem))
      .withSize(1,1)
      .withPosition(1,3);

    Shuffleboard.getTab("GameTab").addBoolean("Note In Pizza Box", ()-> pizzaBoxSubsystem.hasNote)
      .withPosition(0,3);

    Shuffleboard.getTab("GameTab").addBoolean("Arm Running", ()-> !armSubsystem.isEmergencyStop())
      .withSize(1, 1)
      .withPosition(3, 0);

    Shuffleboard.getTab("GameTab").addBoolean("Intake Running", ()-> !intakeSubsystem.isEmergencyStop())
      .withSize(1, 1)
      .withPosition(4, 0);

    Shuffleboard.getTab("GameTab").add("Reset Arm", new ResetArm(armSubsystem, pizzaBoxSubsystem) )
      .withSize(1,1)
      .withPosition(3,1);
    Shuffleboard.getTab("GameTab").add("Reset Intake", new IntakeResetArm(intakeSubsystem))
      .withSize(1,1)
      .withPosition(4,1);
    Shuffleboard.getTab("GameTab").add("IntakeRejectNote", new IntakeRejectNote(intakeSubsystem))
      .withPosition(4,2);
    Shuffleboard.getTab("GameTab").add("Extend Reaction Bar", new Extend(reactionSubsystem))
      .withPosition(4, 4);
     Shuffleboard.getTab("GameTab").add("Retract Reaction Bar", new Retract(reactionSubsystem))
      .withPosition(5, 4);
    

    

      

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


    //Climb stuff (all in climb tab)
    Shuffleboard.getTab("Climb").addDouble("climb distance left", () -> climbSubsystem.getPositionLeft()).withPosition(0, 0);
    Shuffleboard.getTab("Climb").addDouble("climb distance right", () -> climbSubsystem.getPositionRight()).withPosition(1, 0);
    Shuffleboard.getTab("Climb").addBoolean("bot left limit", () -> climbSubsystem.getBotLeftLimit()).withPosition(2, 1);
    Shuffleboard.getTab("Climb").addBoolean("bot right limit", () -> climbSubsystem.getBotRightLimit()).withPosition(3, 1);
    Shuffleboard.getTab("Climb").addBoolean("top left limit", () -> climbSubsystem.getTopLeftLimit()).withPosition(2, 0);
    Shuffleboard.getTab("Climb").addBoolean("top right limit", () -> climbSubsystem.getTopRightLimit()).withPosition(3, 0);

    Shuffleboard.getTab("Climb").add("motor down", new MotorDown(climbSubsystem, swerve)).withPosition(1, 1);
    Shuffleboard.getTab("Climb").add("motor up", new MotorUp(climbSubsystem, swerve)).withPosition(0, 1);

    Shuffleboard.getTab("Climb").add("climb prep", new PrepClimb(climbSubsystem, swerve, armSubsystem, reactionSubsystem)).withPosition(4, 0);
    Shuffleboard.getTab("Climb").add("climb & shoot", new ClimbAndShoot(climbSubsystem, swerve, armSubsystem, pizzaBoxSubsystem)).withPosition(5, 0);
    Shuffleboard.getTab("Climb").add("unclimb1", new UnClimb(climbSubsystem, swerve, armSubsystem, pizzaBoxSubsystem)).withPosition(4, 1);
    Shuffleboard.getTab("Climb").add("unclimb2", new UnClimbPartTwoThatWillBringDownTheMotor(climbSubsystem, swerve, armSubsystem, reactionSubsystem)).withPosition(5, 1);

    Shuffleboard.getTab("Climb").add("STOP CLIMB!!!!", new StopClimb(climbSubsystem)).withSize(2, 1).withPosition(2, 2);

    Shuffleboard.getTab("Climb").addDouble("Reaction Bar Angle", ()-> reactionSubsystem.getPos()).withPosition(9, 4);


    //Shuffleboard.getTab("System Check")
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
