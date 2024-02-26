// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testcontainers;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.commands.LimeLight.Align;
import frc.robot.commands.LimeLight.FaceAprilTag;
import frc.robot.commands.LimeLight.Forward;
import frc.robot.commands.LimeLight.Sideways;
import frc.robot.commands.driveCommands.rotateinPlace;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.time.Instant;

import frc.robot.subsystems.LimeVision.ApriltagController;
import frc.robot.subsystems.LimeVision.LimeLightSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class VisionContainer implements BaseContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  private final LimeLightSub limeLightSub;
  private final ShuffleboardTab limeLightTab;
  private final ApriltagController apriltagController;

  CommandXboxController driverXbox = new CommandXboxController(0);

  public String getDriveTrainName(){
    return "swerve/ryker_falcon";
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public VisionContainer()
  {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         getDriveTrainName()));                       
    limeLightSub = new LimeLightSub("limelight");
    limeLightTab = Shuffleboard.getTab("Limelight");
                                                                         
    apriltagController = new ApriltagController(drivebase, limeLightSub); 
                                                                      
    // graphs sideways error
    limeLightTab.addDouble("Sideways Output", ()->apriltagController.getErrorSideways())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);
    limeLightTab.addDouble("Forward Output", ()->apriltagController.getErrorForward())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);
    // limeLightTab.addDouble("Rotation Output", ()->apriltagController.getErrorRotation())
    //   .withWidget(BuiltInWidgets.kGraph)
    //   .withSize(3,3)
    //   .withPosition(3, 0);
    // limeLightTab.addDouble("Forward TA Output", ()->apriltagController.getErrorForwardTA())
    //   .withWidget(BuiltInWidgets.kGraph)
    //   .withSize(3,3)
    //   .withPosition(3, 0);
    
    // limeLightTab.addDouble("Robot Heading", ()->apriltagController.getRobotHeading())
    //   .withWidget(BuiltInWidgets.kGraph)
    //   .withSize(3,3)
    //   .withPosition(3, 0);

      limeLightTab.addDouble("Rotate Error", ()-> rotateinPlace.angleRate)
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);

    // limeLightTab.addNumber("Distance Bot Pose", ()-> apriltagController.getBotPoseDistance());
    limeLightTab.addNumber("Distance X Input", ()-> apriltagController.getDistanceForward());
    // limeLightTab.addNumber("Distance Y Input", ()-> apriltagController.getDistanceSideways());
    // limeLightTab.addNumber("Distance Tx Input", ()-> limeLightSub.getTx());
    limeLightTab.addNumber("Distance X Output", ()-> apriltagController.getErrorForward());
    // limeLightTab.addNumber("Distance Y Output", ()-> apriltagController.getErrorSideways());
    // limeLightTab.addNumber("Distance Tx Output", ()-> apriltagController.getErrorRotation());
    // limeLightTab.addNumber("TA Size", ()-> limeLightSub.getTa());
    // limeLightTab.addNumber("TA Distance", ()-> limeLightSub.getTaDistance());

    limeLightTab.addNumber("Current Robot Heading", ()-> apriltagController.getRobotHeading());
    limeLightTab.addNumber("Get Tag Heading", ()-> apriltagController.getApriltagHeading());

    limeLightTab.addNumber("Robot Pose X", ()-> drivebase.getPose().getX());
    limeLightTab.addNumber("Robot Pose Y", ()-> drivebase.getPose().getY());

    // limeLightSub.addNumber("TA", ()-> limeLightSub.getTa());

    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis(), () -> true);

    drivebase.setDefaultCommand(closedFieldRel);  //TO CHANGE DRIVE BASE

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));    
    // driverXbox.x().onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    // driverXbox.y().onTrue(new FaceAprilTag(drivebase, apriltagController));
    // driverXbox.a().onTrue(new Sideways(drivebase, apriltagController));

    // driverXbox.y().onTrue(new rotateinPlace(() -> 90, drivebase));
    driverXbox.b().onTrue(new FaceAprilTag(drivebase, apriltagController));
    driverXbox.y().onTrue(new InstantCommand(drivebase::resetStartPos));
    driverXbox.x().onTrue(new Forward(drivebase, apriltagController));

    driverXbox.a().onTrue(new Align(drivebase, apriltagController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand()
  // {
  //   // An example command will be run in autonomous
  //   return drivebase.getAutonomousCommand("New Path", true);
  // }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}