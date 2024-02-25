// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testcontainers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.commands.LimeLight.FaceAprilTag;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.commands.LimeLight.FaceAprilTag;
import frc.robot.commands.LimeLight.Sideways;
import frc.robot.commands.LimeLight.DriveToTag;
import frc.robot.commands.LimeLight.AddVisionData;
import frc.robot.commands.LimeLight.DriveThere;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class VisionContainer implements BaseContainer
{

  // The robot's subsystems and commands are defined here...
  public static SwerveSubsystem drivebase;
  private final SendableChooser<Command> autoChooser;

  CommandXboxController driverXbox = new CommandXboxController(0);

  // limelight
  private final LimeLightSub limeLightSub;

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
    autoChooser = AutoBuilder.buildAutoChooser();
    drivebase.resetOdometry(new Pose2d(2.5, 5.53,new Rotation2d(180)));
    drivebase.zeroGyro();
    

    limeLightSub = new LimeLightSub("limelight" );                                                                   
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedFieldRel = new TeleopDrive( 
        drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis(), () -> true);

    TeleopDrive autoAlignVision = new TeleopDrive(
        drivebase,
        () -> 0,
        () -> 0, 
        () -> -limeLightSub.getError(), () -> true);
        
    
    drivebase.setDefaultCommand(closedFieldRel);  //TO CHANGE DRIVE BASE
    // CommandScheduler.getInstance().schedule(new AddVisionData(drivebase, limeLightSub));
    

    ShuffleboardTab driveTrainShuffleboardTab = Shuffleboard.getTab("Drive Train");

    driveTrainShuffleboardTab.addDouble("X Position", ()->drivebase.getPose().getX())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(0, 0);
    driveTrainShuffleboardTab.addDouble("Y Position", ()->drivebase.getPose().getY())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);
    driveTrainShuffleboardTab.addDouble("Angle", ()->drivebase.getPose().getRotation().getDegrees())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(6, 0);
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
    driverXbox.x().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    
    // points to AprilTag
    driverXbox.a().onTrue(new DriveToTag(drivebase, limeLightSub, () -> false));
    // driverXbox.y().onTrue(new AddVisionData(drivebase, limeLightSub));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
   return autoChooser.getSelected();
  }
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