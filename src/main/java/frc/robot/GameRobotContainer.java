package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.MotorDown;
import frc.robot.commands.ClimberCommands.MotorUp;
import frc.robot.commands.Arm.ArmEmergencyStop;
import frc.robot.commands.Arm.ScoreInAmp;
import frc.robot.commands.Arm.ScoreInSpeakerAdjustable;
import frc.robot.commands.Arm.ScoreInSpeakerHigh;
import frc.robot.commands.Arm.ScoreInSpeakerUnderHand;
import frc.robot.commands.Arm.ScoreInTrap;
import frc.robot.commands.Arm.SpinAndSwing;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.Arm.TestingOnlyShoot;
import frc.robot.commands.IntakeCommands.IntakeEmergencyStop;
import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakeRejectNote;
import frc.robot.commands.IntakeCommands.PickUpFromGroundAndPassToPizzaBox;
import frc.robot.commands.IntakeCommands.SpinIntakePID;
import frc.robot.commands.ledcommands.ChangeLEDColor;
import frc.robot.commands.ledcommands.ChangeLEDToBlue;
import frc.robot.commands.ledcommands.ChangeLEDToGreen;
import frc.robot.commands.ledcommands.ChangeLEDToRed;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
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
    private final Climbsubsystem m_Climbsubsystem;

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

        LEDSubsystem led = new LEDSubsystem(Constants.LEDConstants.ledPortNumber);

         m_Climbsubsystem = new Climbsubsystem( ClimbConstants.MOTOR_LEFT_ID, 
                                                        ClimbConstants.MOTOR_RIGHT_ID, 
                                                        ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                        ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio"); //change arguments


        TeleopDrive closedFieldRel = new TeleopDrive(
        m_drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);

        m_drivebase.setDefaultCommand(closedFieldRel);

        configureBindings();
        configurePathPlannerCommands();

        autoChooser = AutoBuilder.buildAutoChooser("0-S(Amp)-0");
        Shuffleboard.getTab("Autonomous").add("Autonomous Chooser", autoChooser).withSize(2, 1);


        ShuffleboardTab driveTrainShuffleboardTab = Shuffleboard.getTab("Drive Train");
    
    driveTrainShuffleboardTab.addDouble("X Position", ()->m_drivebase.getPose().getX())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(0, 0);
    driveTrainShuffleboardTab.addDouble("Y Position", ()->m_drivebase.getPose().getY())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(3, 0);
    driveTrainShuffleboardTab.addDouble("Angel", ()->m_drivebase.getPose().getRotation().getDegrees())
      .withWidget(BuiltInWidgets.kGraph)
      .withSize(3,3)
      .withPosition(6, 0);




    }


    private void configureBindings() {
      final PIDController intakeController = new PIDController(.005, .0, .0);
      intakeController.setTolerance(Constants.IntakeConstants.TOLERANCE);
      
      driverController.x().onTrue(new SpinToArmAngle(m_ArmSubsystem, 240));

      driverController.y().onTrue(new TestingOnlyShoot(m_PizzaBoxSubsystem, m_ArmSubsystem, 150));

      // driverController.x().onTrue(new SpinIntakePID(m_IntakeSubsystem, 0));
      // driverController.y().onTrue(new SpinIntakePID(m_IntakeSubsystem, 70));
      driverController.a().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));
      //driverController.b().onTrue(new IntakePassNoteToPizzaBox(m_IntakeSubsystem, m_PizzaBoxSubsystem));
      driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));
      driverController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem));
      driverController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));
      driverController.b().onTrue(new IntakeRejectNote(m_IntakeSubsystem));
      
      //This should be a parallel command with other stuff
     /* / driverController.x().onTrue(new ChangeLEDToBlue(led));//pressing x on the controller runs a
      driverController.y().onTrue(new ChangeLEDToRed(led));
      driverController.b().onTrue(new ChangeLEDToGreen(led));
      driverController.a().onTrue(new ChangeLEDColor(led, 255, 0, 255));
      driverController.rightBumper().onTrue(new ChangeLEDColor(led, 0, 0, 0));
      */

      
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

