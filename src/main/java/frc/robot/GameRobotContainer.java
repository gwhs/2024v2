package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
    private final SwerveSubsystem m_drivebase;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final PizzaBoxSubsystem m_PizzaBoxSubsystem;
    //private final LEDSubsystem m_ledsubsystem;
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

        //m_ledsubsystem = new LEDSubsystem(Constants.LEDConstants.ledPortNumber);

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

        TeleopDrive closedFieldRel = new TeleopDrive(
        m_drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);

        m_drivebase.setDefaultCommand(closedFieldRel);
        configureBindings();
    }


    private void configureBindings() {      
      
      driverController.y().onTrue(new ScoreInSpeakerHigh(m_PizzaBoxSubsystem, m_ArmSubsystem));
      driverController.a().onTrue(new ScoreInAmp(m_PizzaBoxSubsystem, m_ArmSubsystem)); 
      driverController.b().onTrue(new PickUpFromGroundAndPassToPizzaBox(m_PizzaBoxSubsystem,m_ArmSubsystem, m_IntakeSubsystem));
      driverController.x().whileTrue(new DecreaseSpeed(closedFieldRel));

      driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));

      
      operatorController.rightBumper().onTrue(new ArmEmergencyStop(m_ArmSubsystem));
      operatorController.leftBumper().onTrue(new IntakeEmergencyStop(m_IntakeSubsystem));

      // operatorController.a().whileTrue(new MotorUp(m_Climbsubsystem, m_drivebase));
      // operatorController.b().whileTrue(new MotorDown(m_Climbsubsystem, m_drivebase));
      // operatorController.x().onTrue(new ScoreInTrap(m_PizzaBoxSubsystem, m_ArmSubsystem));
      // operatorController.y().onTrue(new SpinToArmAngle(m_ArmSubsystem, 135));
      
    }

    public void setMotorBrake(boolean brake)
    {
      m_drivebase.setMotorBrake(brake);
    }
}

