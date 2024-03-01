// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testcontainers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.commands.driveCommands.DecreaseSpeed;
import frc.robot.commands.driveCommands.rotateinPlace;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class DriveContainer implements BaseContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  private TeleopDrive closedFieldRel;

  CommandXboxController driverController = new CommandXboxController(1);
  CommandXboxController driverXbox = new CommandXboxController(0);

  //private final SendableChooser<Command> autoChooser;

  public String getDriveTrainName(){
    return "swerve/hajel_kraken";
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public DriveContainer()
  {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         getDriveTrainName()));


    // Configure the trigger bindings
    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driverXbox.getLeftTriggerAxis(),
                                                          () -> -driverXbox.getRightTriggerAxis());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                                                    OperatorConstants.LEFT_X_DEADBAND),
                                                                         () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                                                      OperatorConstants.LEFT_Y_DEADBAND),
                                                                         () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis());

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                      () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                      () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND),
                                                                      () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                  OperatorConstants.RIGHT_X_DEADBAND), 
                                                                      () -> driverXbox.y().getAsBoolean(), 
                                                                      () -> driverXbox.a().getAsBoolean(),
                                                                      () -> driverXbox.x().getAsBoolean(),
                                                                      () -> driverXbox.b().getAsBoolean());

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                    () -> driverXbox.getRawAxis(2), () -> true);
     closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis(), () -> true);

          configureBindings();
     

    drivebase.setDefaultCommand(closedFieldRel);  //TO CHANGE DRIVE BASE
    //drivebase.test();


    // ShuffleboardTab driveTrainShuffleboardTab = Shuffleboard.getTab("Drive Train");
    // ShuffleboardTab angleTab = Shuffleboard.getTab("Theta");

    // SmartDashboard.putData("Rotate To Speaker", new rotateinPlace(()->UtilMath.BLUESpeakerTheta(drivebase.getPose()), drivebase));

    // angleTab.addDouble("Estimated Theta", ()->UtilMath.BLUESpeakerTheta(drivebase.getPose()));

    ShuffleboardTab angleTest = Shuffleboard.getTab("Rotate In Place");
    SmartDashboard.putData("Rotate theta", new rotateinPlace(()->180, drivebase));
  
    


ShuffleboardTab driveTrainShuffleboardTab = Shuffleboard.getTab("Drive Train");
    
    driveTrainShuffleboardTab.addDouble("X Position", ()->drivebase.getPose().getX())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(0, 0);
    driveTrainShuffleboardTab.addDouble("Y Position", ()->drivebase.getPose().getY())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);
    driveTrainShuffleboardTab.addDouble("Angel", ()->drivebase.getPose().getRotation().getDegrees())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(6, 0);


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
    driverXbox.leftBumper().whileTrue(new DecreaseSpeed(closedFieldRel));

    
    

    // driverXbox.a().onTrue(
    //   new rotateinPlace(()-> UtilMath.SpeakerTheta(drivebase.getPose()), drivebase)
    // );
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

    /** This function is called once when the robot is first started up. */
    
    public void simulationInit() {}
  
    /** This function is called periodically whilst in simulation. */
    public void simulationPeriodic() {
     
    }
  }