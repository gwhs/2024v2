package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Reaction.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.Telemetry;
import frc.robot.subsystems.swervedrive.TunerConstants;

import com.pathplanner.lib.auto.AutoBuilder;

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    private final SendableChooser<Command> autoChooser;
  
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final PizzaBoxSubsystem m_PizzaBoxSubsystem;
    private final Climbsubsystem m_ClimbSubsystem;
    private final ReactionSubsystem m_ReactionSubsystem = new ReactionSubsystem();
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

    private final CTRETeleopDrive drive = new CTRETeleopDrive(driverController);
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    public GameRobotContainer() {
        m_IntakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");

        m_ArmSubsystem = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "rio", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        m_PizzaBoxSubsystem = new PizzaBoxSubsystem(PizzaBoxSubsystem.PIZZABOX_ID, 
                    "rio", PizzaBoxSubsystem.SERVO_PWN_SLOT, PizzaBoxSubsystem.SERVO2_PWN_SLOT);

         m_ClimbSubsystem = new Climbsubsystem( Constants.ClimbConstants.MOTOR_LEFT_ID, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_ID, 
                                                Constants.ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                Constants.ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio");

        autoChooser = AutoBuilder.buildAutoChooser("Hajel middle bottom 2");

        drivetrain.setDefaultCommand(drive);
        configureBindings();
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }


    private void configureBindings() {

      /* Driver Controller */
      //RE-ENABLE BUTTONS;
      
      // driverController.y().onTrue(new ScoreInSpeakerUnderHand(m_PizzaBoxSubsystem, m_ArmSubsystem));
      // driverController.a().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem, drive)); 
      // //driverController.b().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem, 10));
      // driverController.x().whileTrue(new DecreaseSpeed(drive));

      // driverController.rightStick().onTrue(new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
      // // driverController.back().onTrue(new ChangeRobotOrientation(closedFieldRel));

      // //driverController.rightBumper().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, ()->UtilMath.overhand.get(UtilMath.distanceFromSpeaker(()->m_drivebase.getPose()))));
      // //driverController.leftBumper().onTrue(new FaceSpeaker(closedFieldRel));

      // //driverController.rightBumper().onTrue(new Aimbot(closedFieldRel, m_ArmSubsystem, m_PizzaBoxSubsystem, m_drivebase));
      // driverController.leftBumper().onTrue(new FaceAmp(drive));

      // //driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));
      // driverController.start().onTrue(new InstantCommand(drivetrain::setHeading));

      // /* Operator Controllers */

      // operatorController.y().onTrue(new PrepClimb(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem, m_PizzaBoxSubsystem));
      // operatorController.b().onTrue(new ClimbAndShoot(m_ClimbSubsystem, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem));
      // operatorController.a().onTrue(new UnClimb(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem));
      // operatorController.x().onTrue(new UnClimbPartTwoThatWillBringDownTheMotor(m_ClimbSubsystem, m_ArmSubsystem, m_ReactionSubsystem, m_PizzaBoxSubsystem));
      // operatorController.start().onTrue(new StopClimb(m_ClimbSubsystem));

      // operatorController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem, m_PizzaBoxSubsystem));
      // operatorController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));

      // operatorController.leftStick().whileTrue(new LockHeadingToSourceForIntake(drive, m_ArmSubsystem, m_PizzaBoxSubsystem));
      // // GenericEntry s = Shuffleboard.getTab("Arm").add("Angle", 236).getEntry();
      // // operatorController.rightStick().onTrue(new ScoreInSpeakerAdjustable(m_PizzaBoxSubsystem, m_ArmSubsystem, ()->s.getDouble(236)));
      // operatorController.rightStick().whileTrue(new FaceSpeaker(drive));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


}

