package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.UtilMath;
import frc.robot.commands.SystemCheck;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ClimbParts.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Telemetry;
import frc.robot.subsystems.swervedrive.TunerConstants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private final SendableChooser<Command> autoChooser;
  
    //private final SwerveSubsystem m_drivebase;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final PizzaBoxSubsystem m_PizzaBoxSubsystem;
    //private final LEDSubsystem m_ledsubsystem;
    private final Climbsubsystem m_ClimbSubsystem;
    private final ReactionSubsystem m_ReactionSubsystem;
    //private final LimeLightSub m_LimelightSubsystem;

    //private final TeleopDrive closedFieldRel;

    /*CTRE stuff */


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

    private final CTRETeleopDrive drive = new CTRETeleopDrive(driverController);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    public String getDriveTrainName(){
      return "swerve/hajel_kraken";
    }


    public GameRobotContainer() {
        // m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        //                                                                  getDriveTrainName()));
        m_IntakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");

        m_ArmSubsystem = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "rio", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        m_PizzaBoxSubsystem = new PizzaBoxSubsystem(PizzaBoxSubsystem.PIZZABOX_ID, 
                    "rio", PizzaBoxSubsystem.SERVO_PWN_SLOT, PizzaBoxSubsystem.SERVO2_PWN_SLOT);

        //m_ledsubsystem = new LEDSubsystem(Constants.LEDConstants.ledPortNumber);

         m_ClimbSubsystem = new Climbsubsystem( Constants.ClimbConstants.MOTOR_LEFT_ID, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_ID, 
                                                Constants.ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio");
        
        //m_LimelightSubsystem  = new LimeLightSub("limelight", m_drivebase); 

        m_ReactionSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);

        // closedFieldRel = new TeleopDrive(
        //                                   m_drivebase,
        //                                   () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        //                                   () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        //                                   () -> (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), OperatorConstants.ROTATION_DEADBAND) - MathUtil.applyDeadband(driverController.getRightTriggerAxis(), OperatorConstants.ROTATION_DEADBAND)), 
        //                                   () -> true);



        configurePathPlannerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Hajel middle bottom 2");


        //m_drivebase.setDefaultCommand(closedFieldRel);
        drivetrain.setDefaultCommand(drive);
        // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
        // .withDeadband(5 * 0.1).withRotationalDeadband(2 * 0.1) // Add a 10% deadband
        // .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(-driverController.getLeftY() * 5) // Drive forward with
        //                                                                                    // negative Y (forward)
        //     .withVelocityY(-driverController.getLeftX() * 5) // Drive left with negative X (left)
        //     .withRotationalRate(-driverController.getRightX() * 2) // Drive counterclockwise with negative X (left)
        // ));


        //SetupShuffleboard.setupShuffleboard(m_drivebase, m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem, m_LimelightSubsystem, m_ClimbSubsystem, m_ReactionSubsystem, autoChooser, closedFieldRel);
      Shuffleboard.getTab("System Check").add("check", new SystemCheck(m_ArmSubsystem, m_ClimbSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem, drive));
      Shuffleboard.getTab("GameTab").add("Reset Arm", new ResetArm(m_ArmSubsystem, m_PizzaBoxSubsystem))
        .withPosition(3,1);
      Shuffleboard.getTab("GameTab").add("Reset Intake", new IntakeResetArm(m_IntakeSubsystem))
        .withPosition(4,1);
      Shuffleboard.getTab("GameTab").add("Autonomous Chooser", autoChooser)
        .withSize(2, 1)
        .withPosition(0, 2);
      DataLogManager.log("rotate in place P: " + Constants.DriveConstants.kP);
      DataLogManager.log("rotate in place I: " + Constants.DriveConstants.kI);
      DataLogManager.log("rotate in place D: " + Constants.DriveConstants.kD);
      Shuffleboard.getTab("LogBooleans").addBoolean("isFaceSpeaker", ()-> drive.isFaceSpeaker);
      Shuffleboard.getTab("LogBooleans").addBoolean("isBackSpeaker", ()-> drive.isBackSpeaker);
      Shuffleboard.getTab("LogBooleans").addBoolean("faceAmp", ()-> drive.faceAmp);
      Shuffleboard.getTab("LogBooleans").addBoolean("isSlow", ()-> drive.isSlow);
      Shuffleboard.getTab("LogBooleans").addBoolean("isHeadingLock", ()-> drive.isHeadingLock);
      Shuffleboard.getTab("LogBooleans").addBoolean("faceSpeaker", ()-> drive.faceSpeaker);
      Shuffleboard.getTab("GameTab").addBoolean("Arm Running", ()-> !m_ArmSubsystem.isEmergencyStop())
        .withPosition(3, 0);
      Shuffleboard.getTab("GameTab").addBoolean("Intake Running", ()-> !m_IntakeSubsystem.isEmergencyStop())
        .withPosition(4, 0);
        
        configureBindings();
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }


    private void configureBindings() {

      /* Driver Controller */
      //RE-ENABLE BUTTONS;
      
      driverController.y().onTrue(new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));
      driverController.a().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem, drive)); 
      driverController.b().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem, 10));
      driverController.x().whileTrue(new DecreaseSpeed(drive));

      driverController.rightStick().onTrue(new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
      // driverController.back().onTrue(new ChangeRobotOrientation(closedFieldRel));

      //driverController.rightBumper().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, ()->UtilMath.overhand.get(UtilMath.distanceFromSpeaker(()->m_drivebase.getPose()))));
      //driverController.leftBumper().onTrue(new FaceSpeaker(closedFieldRel));

      //driverController.rightBumper().onTrue(new Aimbot(closedFieldRel, m_ArmSubsystem, m_PizzaBoxSubsystem, m_drivebase));
      driverController.leftBumper().onTrue(new FaceAmp(drive));

      //driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));
      driverController.start().onTrue(new InstantCommand(drivetrain::setHeading));

      /* Operator Controllers */

      operatorController.y().onTrue(new PrepClimb(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem, m_PizzaBoxSubsystem));
      operatorController.b().onTrue(new ClimbAndShoot(m_ClimbSubsystem, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem));
      operatorController.a().onTrue(new UnClimb(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem));
      operatorController.x().onTrue(new UnClimbPartTwoThatWillBringDownTheMotor(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem, m_PizzaBoxSubsystem));
      operatorController.start().onTrue(new StopClimb(m_ClimbSubsystem));

      operatorController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem, m_PizzaBoxSubsystem));
      operatorController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));

      operatorController.leftStick().whileTrue(new LockHeadingToSourceForIntake(drive, m_ArmSubsystem, m_PizzaBoxSubsystem));
      // GenericEntry s = Shuffleboard.getTab("Arm").add("Angle", 236).getEntry();
      // operatorController.rightStick().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, ()->s.getDouble(236)));
      operatorController.rightStick().whileTrue(new FaceSpeaker(drive));
    }

    private void configurePathPlannerCommands() {
    
      NamedCommands.registerCommand("Intake", new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));

      NamedCommands.registerCommand("Speaker (underhand)", new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));

      NamedCommands.registerCommand("Speaker (A1)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 248)/*, 
                                                                                  new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)*/));
      NamedCommands.registerCommand("Speaker (A2)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245)/*, 
                                                                                  new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)*/));
      NamedCommands.registerCommand("Speaker (A3)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 250)/*, 
                                                                                  new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)*/));

      NamedCommands.registerCommand("Speaker (S1)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241));
      NamedCommands.registerCommand("Speaker (S2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 236));
      NamedCommands.registerCommand("Speaker (S3)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241));

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();

    // return new WaitCommand(.1)
    //         .andThen(new IntakeResetArm(m_IntakeSubsystem).withTimeout(3)
    //         .alongWith(new Retract(m_ReactionSubsystem)))
    //         .andThen(autoChooser.getSelected());
  }

  public Command teleopInitReset() {
    return new Retract(m_ReactionSubsystem).withTimeout(0.5)
           //.andThen(new IntakeResetArm(m_IntakeSubsystem)).withTimeout(3)
           .alongWith(new ResetArm(m_ArmSubsystem, m_PizzaBoxSubsystem).withTimeout(3));
  }

}

