package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands.MotorDown;
import frc.robot.commands.ClimberCommands.MotorUp;
import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
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

public class GameRobotContainer implements BaseContainer {

    CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    CommandXboxController OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
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

        configureBindings();

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
        
        driverController.x().onTrue(new SpinIntakePID(m_IntakeSubsystem, 0));
        driverController.y().onTrue(new SpinIntakePID(m_IntakeSubsystem, 106));
        driverController.a().onTrue(new IntakePickUpFromGround(m_IntakeSubsystem));
        driverController.b().onTrue(new IntakePassNoteToPizzaBox(m_IntakeSubsystem, m_PizzaBoxSubsystem));
        driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));

         OperatorController.a().whileTrue(new MotorUp(m_Climbsubsystem, m_drivebase));
        OperatorController.b().whileTrue(new MotorDown(m_Climbsubsystem, m_drivebase));
        
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

