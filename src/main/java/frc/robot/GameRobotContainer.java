package frc.robot;

import java.io.File;

import javax.security.sasl.AuthorizeCallback;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.ClimberCommands.ClimbParts.ClimbAndShoot;
import frc.robot.commands.ClimberCommands.ClimbParts.PrepClimb;
import frc.robot.commands.ClimberCommands.ClimbParts.UnClimb;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.networktables.GenericEntry;

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
        m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         getDriveTrainName()));
        m_IntakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");

        m_ArmSubsystem = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "CAN_Network", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        m_PizzaBoxSubsystem = new PizzaBoxSubsystem(PizzaBoxSubsystem.PizzaBox.PIZZABOX_ID, 
                    "rio", PizzaBoxSubsystem.PizzaBox.SERVO_PWN_SLOT);

        //m_ledsubsystem = new LEDSubsystem(Constants.LEDConstants.ledPortNumber);

         m_ClimbSubsystem = new Climbsubsystem( Constants.ClimbConstants.MOTOR_LEFT_ID, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_ID, 
                                                Constants.ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio");
        
        m_LimelightSubsystem  = new LimeLightSub("limelight", m_drivebase); 
          closedFieldRel = new TeleopDrive(
                                            m_drivebase,
                                            () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
                                            () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
                                            () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);
                                                  

        m_ReactionSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);

        autoChooser = AutoBuilder.buildAutoChooser("");

        m_drivebase.setDefaultCommand(closedFieldRel);

        SetupShuffleboard.setupShuffleboard(m_drivebase, m_PizzaBoxSubsystem, m_ArmSubsystem, m_IntakeSubsystem, m_LimelightSubsystem, m_ClimbSubsystem, m_ReactionSubsystem, autoChooser);

        configureBindings();
        configurePathPlannerCommands();


    }


    private void configureBindings() {
      
      driverController.y().onTrue(new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
      driverController.a().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem)); 
      driverController.b().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));
      driverController.x().whileTrue(new DecreaseSpeed(closedFieldRel));

      // driverController.b().onTrue(new SpinIntakePID(m_IntakeSubsystem, 0));
      // driverController.x().onTrue(new SpinIntakePID(m_IntakeSubsystem, 77));

      driverController.rightBumper().onTrue(new BackSpeaker(closedFieldRel));
      driverController.leftBumper().onTrue(new FaceSpeaker(closedFieldRel));


      driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));


      operatorController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem));
      operatorController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));

      operatorController.y().onTrue(new PrepClimb(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_ReactionSubsystem));
      operatorController.b().onTrue(new ClimbAndShoot(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem));
      operatorController.a().onTrue(new UnClimb(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem));
      //operatorController.().onTrue(new MotorDown(m_Climbsubsystem, m_drivebase));
      operatorController.x().onTrue(new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));

      driverController.leftBumper().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, Shuffleboard.getTab("GameTab").add("Angle", 242).getEntry().getDouble(245)));
    }

    private void configurePathPlannerCommands() { //register rest of commands when get them
    
    NamedCommands.registerCommand("Wait (half a second)", new WaitCommand(0.5));
    NamedCommands.registerCommand("Wait (one second)", new WaitCommand(1));
    NamedCommands.registerCommand("Intake", new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));
    NamedCommands.registerCommand("Amp", new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem));
    NamedCommands.registerCommand("Speaker", new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
    NamedCommands.registerCommand("Speaker (underhand)", new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));
    NamedCommands.registerCommand("Speaker (subwoofer)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 227.22));
    NamedCommands.registerCommand("Speaker (A2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 227.81));
    NamedCommands.registerCommand("Speaker (S3)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 229.48));
    NamedCommands.registerCommand("Speaker (A3-A2)", new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, 225.37));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Path", true);
    return autoChooser.getSelected();
  }

    public void setMotorBrake(boolean brake)
    {
      m_drivebase.setMotorBrake(brake);
    }
}

