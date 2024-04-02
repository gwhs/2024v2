package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.UtilMath;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ClimbParts.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private final SendableChooser<Command> autoChooser;
  
    private final SwerveSubsystem m_drivebase;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final PizzaBoxSubsystem m_PizzaBoxSubsystem;
    //private final LEDSubsystem m_ledsubsystem;
    private final Climbsubsystem m_ClimbSubsystem;
    private final ReactionSubsystem m_ReactionSubsystem;
    private final LimeLightSub m_LimelightSubsystem;

    private final TeleopDrive closedFieldRel;

    public String getDriveTrainName(){
      return "swerve/hajel_kraken";
    }


    public GameRobotContainer() {
    //       LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    // Shuffleboard.getTab("GameTab").addCamera("Vision", "limelight", "http://limelight.local:5800").withSize(4,3).withPosition(5, 0);
        m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         getDriveTrainName()));
        m_IntakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");

        m_ArmSubsystem = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "CAN_Network", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        m_PizzaBoxSubsystem = new PizzaBoxSubsystem(PizzaBoxSubsystem.PizzaBox.PIZZABOX_ID, 
                    "rio", PizzaBoxSubsystem.PizzaBox.SERVO_PWN_SLOT, PizzaBoxSubsystem.PizzaBox.SERVO_PWN_SLOT_THE_SECOND);

        //m_ledsubsystem = new LEDSubsystem(Constants.LEDConstants.ledPortNumber);

         m_ClimbSubsystem = new Climbsubsystem( Constants.ClimbConstants.MOTOR_LEFT_ID, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_ID, 
                                                Constants.ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio");
        
        m_LimelightSubsystem  = new LimeLightSub("limelight", m_drivebase); 

        m_ReactionSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);

   closedFieldRel = new TeleopDrive(
                                          m_drivebase,
                                          () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
                                          () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
                                          () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);



        configurePathPlannerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("2N-S2-A3");


        m_drivebase.setDefaultCommand(closedFieldRel);

        //SetupShuffleboard.setupShuffleboard(m_drivebase, m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem, m_LimelightSubsystem, m_ClimbSubsystem, m_ReactionSubsystem, autoChooser, closedFieldRel);

        Shuffleboard.getTab("Arm").add("270", new SpinToArmAngle(m_ArmSubsystem, 270));
        Shuffleboard.getTab("Arm").add("2nd forward", new SwingForwardServoTheSecond(m_PizzaBoxSubsystem));
        Shuffleboard.getTab("Arm").add("2nd backward", new SwingBackServoTheSecond(m_PizzaBoxSubsystem));


        configureBindings();
        
    }


    private void configureBindings() {

      /* Driver Controller */
      //RE-ENABLE BUTTONS;
      
      driverController.y().onTrue(new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));
      driverController.a().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem)); 
      driverController.b().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem, 10));
      driverController.x().whileTrue(new DecreaseSpeed(closedFieldRel));

      driverController.rightStick().onTrue(new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
      //driverController.leftStick().onTrue(new ChangeRobotOrientation(closedFieldRel));

      //driverController.rightBumper().onTrue(new BackSpeaker(closedFieldRel));
      //driverController.leftBumper().onTrue(new FaceSpeaker(closedFieldRel));

      driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));


      /* Operator Controllers */

      operatorController.y().onTrue(new PrepClimb(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_ReactionSubsystem));
      operatorController.b().onTrue(new ClimbAndShoot(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem));
      operatorController.a().onTrue(new UnClimb(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem));
      operatorController.x().onTrue(new UnClimbPartTwoThatWillBringDownTheMotor(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_ReactionSubsystem));
      operatorController.start().onTrue(new StopClimb(m_ClimbSubsystem));

      operatorController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem, m_PizzaBoxSubsystem));
      operatorController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));

      operatorController.leftStick().whileTrue(new LockHeadingToSourceForIntake(closedFieldRel, m_ArmSubsystem, m_PizzaBoxSubsystem));
      // operatorController.  ?? ().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, Shuffleboard.getTab("Arm").add("Angle", 242).getEntry().getDouble(245)));
      operatorController.rightStick().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, Shuffleboard.getTab("Arm").add("Angle", 240).getEntry().getDouble(245)));
      
    }

    private void configurePathPlannerCommands() {
    
    NamedCommands.registerCommand("Wait (half a second)", new WaitCommand(0.5));
    NamedCommands.registerCommand("Wait (one second)", new WaitCommand(1));

    NamedCommands.registerCommand("Intake", new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));

    NamedCommands.registerCommand("Amp", new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem));

    NamedCommands.registerCommand("Speaker (underhand)", new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));
    NamedCommands.registerCommand("Speaker (subwoofer)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 227.22));

    NamedCommands.registerCommand("Speaker (A1)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 248)/*, 
                                                                                 new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)*/));
    NamedCommands.registerCommand("Speaker (A2)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 248)/*, 
                                                                                 new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)*/));
    NamedCommands.registerCommand("Speaker (A3)", new ParallelDeadlineGroup(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 248), 
                                                                                 new rotateinPlace(()-> UtilMath.BackSpeakerTheta(m_drivebase.getPose()), m_drivebase)));

    NamedCommands.registerCommand("Speaker (A1-A2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245));
    NamedCommands.registerCommand("Speaker (A2-A3)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245));

    NamedCommands.registerCommand("Speaker (A3-A2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245));
    NamedCommands.registerCommand("Speaker (A2-A1)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245));

    NamedCommands.registerCommand("Speaker (S1)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241));
    NamedCommands.registerCommand("Speaker (S2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 236));
    NamedCommands.registerCommand("Speaker (S3)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241));

    NamedCommands.registerCommand("Speaker (S1) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));
    NamedCommands.registerCommand("Speaker (S2) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 236)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));
    NamedCommands.registerCommand("Speaker (S3) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 241)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));

    NamedCommands.registerCommand("Speaker (A1-A2) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));
    NamedCommands.registerCommand("Speaker (A2-A3) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));

    NamedCommands.registerCommand("Speaker (A3-A2) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));
    NamedCommands.registerCommand("Speaker (A2-A1) then Intake", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 245)
                                  .andThen(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }

  public Command teleopInitReset() {
    return new ResetArm(m_ArmSubsystem, m_PizzaBoxSubsystem) 
           .andThen(new IntakeResetArm(m_IntakeSubsystem));
  }

  public Command autoInitReset() {
    return new IntakeResetArm(m_IntakeSubsystem);
  }
}

