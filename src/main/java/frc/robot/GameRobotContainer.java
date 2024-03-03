package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorDown;
import frc.robot.commands.ClimberCommands.ActuallyMovesMotors.MotorUp;
import frc.robot.commands.ClimberCommands.AutoClimb.ClimbUp;
import frc.robot.commands.ClimberCommands.AutoTrap.Trap;
import frc.robot.commands.ClimberCommands.AutoTrap.TrapNoClimbDown;
import frc.robot.commands.ClimberCommands.ClimbParts.ClimbAndShoot;
import frc.robot.commands.ClimberCommands.ClimbParts.PrepClimb;
import frc.robot.commands.Arm.ArmEmergencyStop;
import frc.robot.commands.Arm.ScoreInAmp;
import frc.robot.commands.Arm.ScoreInTrap;
import frc.robot.commands.Arm.ScoreInTrapStutter;
import frc.robot.commands.Arm.SpinAndSwing;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.SpinToArmAngle;
import frc.robot.commands.Arm.TestingOnlyShoot;
import frc.robot.commands.IntakeCommands.IntakeEmergencyStop;
import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakeRejectNote;
import frc.robot.commands.IntakeCommands.PickUpFromGroundAndPassToPizzaBox;
import frc.robot.commands.IntakeCommands.SpinIntakePID;
import frc.robot.commands.driveCommands.DecreaseSpeed;
import frc.robot.commands.ReactionArmCommands.Extend;
import frc.robot.commands.ReactionArmCommands.Retract;
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
import frc.robot.subsystems.ReactionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
    private final SwerveSubsystem m_drivebase;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final PizzaBoxSubsystem m_PizzaBoxSubsystem;
    private final Climbsubsystem m_ClimbSubsystem;
    private final ReactionSubsystem m_ReactionSubsystem;

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

        m_ClimbSubsystem = new Climbsubsystem( ClimbConstants.MOTOR_LEFT_ID, 
                                                        ClimbConstants.MOTOR_RIGHT_ID, 
                                                        ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                        ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio"); //change arguments
          closedFieldRel = new TeleopDrive(
                                            m_drivebase,
                                            () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
                                            () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
                                            () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);
                                                  

        m_ReactionSubsystem = new ReactionSubsystem(Constants.ReactionConstants.reactionID, Constants.ReactionConstants.reactionCAN);

        configureBindings();


        //All Climb Shuffleboard stuff (all in climb tab)
        Shuffleboard.getTab("Climb").addDouble("climb distance left", () -> m_ClimbSubsystem.getPositionLeft()).withPosition(0, 0);
        Shuffleboard.getTab("Climb").addDouble("climb distance right", () -> m_ClimbSubsystem.getPositionRight()).withPosition(1, 0);
        Shuffleboard.getTab("Climb").addBoolean("bot left limit", () -> m_ClimbSubsystem.getBotLeftLimit()).withPosition(2, 1);
        Shuffleboard.getTab("Climb").addBoolean("bot right limit", () -> m_ClimbSubsystem.getBotRightLimit()).withPosition(3, 1);
        Shuffleboard.getTab("Climb").addBoolean("top left limit", () -> m_ClimbSubsystem.getTopLeftLimit()).withPosition(2, 0);
        Shuffleboard.getTab("Climb").addBoolean("top right limit", () -> m_ClimbSubsystem.getTopRightLimit()).withPosition(3, 0);

        Shuffleboard.getTab("Climb").add("full auto trap", new Trap(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem))
                        .withPosition(4, 0);
        Shuffleboard.getTab("Climb").add("trap no down", new TrapNoClimbDown(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem, m_ReactionSubsystem))
                        .withPosition(5, 0);
        Shuffleboard.getTab("Climb").add("motor down", new MotorDown(m_ClimbSubsystem, m_drivebase)).withPosition(1, 1);
        Shuffleboard.getTab("Climb").add("motor up", new MotorUp(m_ClimbSubsystem, m_drivebase)).withPosition(0, 1);
        Shuffleboard.getTab("Climb").add("climb prep", new PrepClimb(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_ReactionSubsystem)).withPosition(4, 1);
        Shuffleboard.getTab("Climb").add("climb & shoot", new ClimbAndShoot(m_ClimbSubsystem, m_drivebase, m_ArmSubsystem, m_PizzaBoxSubsystem)).withPosition(5, 1);

        Shuffleboard.getTab("Climb").addDouble("Reaction Bar Angle", ()-> m_ReactionSubsystem.getPos());

        
        TeleopDrive closedFieldRel = new TeleopDrive(
        m_drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);

        m_drivebase.setDefaultCommand(closedFieldRel);



    }


    private void configureBindings() {
      final PIDController intakeController = new PIDController(.005, .0, .0);
      intakeController.setTolerance(Constants.IntakeConstants.TOLERANCE);
      
      driverController.x().onTrue(new SpinToArmAngle(m_ArmSubsystem, 240));

      //driverController.y().onTrue(new TestingOnlyShoot(m_PizzaBoxSubsystem, m_ArmSubsystem, 150));
      driverController.y().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem)); 
      // driverController.x().onTrue(new SpinIntakePID(m_IntakeSubsystem, 0));
      // driverController.y().onTrue(new SpinIntakePID(m_IntakeSubsystem, 70));
      //driverController.a().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));
      //driverController.b().onTrue(new IntakePassNoteToPizzaBox(m_IntakeSubsystem, m_PizzaBoxSubsystem));
      driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));
      driverController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem));
      //driverController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));
      //driverController.b().onTrue(new IntakeRejectNote(m_IntakeSubsystem));

      driverController.x().onTrue(new DecreaseSpeed(closedFieldRel));


      OperatorController.a().onTrue(new Extend(m_ReactionSubsystem));
      OperatorController.b().onTrue(new Retract(m_ReactionSubsystem));
      OperatorController.x().onTrue(new ScoreInTrapStutter(m_PizzaBoxSubsystem, m_ArmSubsystem));
      OperatorController.y().onTrue(new SpinToArmAngle(m_ArmSubsystem, 135));
      
      //This should be a parallel command with other stuff
     /* / driverController.x().onTrue(new ChangeLEDToBlue(led));//pressing x on the controller runs a
      driverController.y().onTrue(new ChangeLEDToRed(led));
      driverController.b().onTrue(new ChangeLEDToGreen(led));
      driverController.a().onTrue(new ChangeLEDColor(led, 255, 0, 255));
      driverController.rightBumper().onTrue(new ChangeLEDColor(led, 0, 0, 0));
      */
    }

    public void setMotorBrake(boolean brake)
    {
      m_drivebase.setMotorBrake(brake);
    }
}

